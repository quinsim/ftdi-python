# Standard library imports
from __future__ import annotations
import enum
import time
import math
from typing import Generator
from contextlib import contextmanager

# Third party imports

# Local application imports
from ftdi_python.ftd2xx import FTD2XX, Bitmodes, FtdiDevice, OpenExFlag
from ftdi_python.mpsse_constants import *
from ftdi_python.exceptions import I2cNackError

# fmt: off
ADBUS0 = ACBUS0 = CLK  = SCL   = 0b00000001
ADBUS1 = ACBUS1 = MOSI = SDA_0 = 0b00000010
ADBUS2 = ACBUS2 = MISO = SDA_1 = 0b00000100
ADBUS3 = ACBUS3 = CS           = 0b00001000
ADBUS4 = ACBUS4                = 0b00010000
ADBUS5 = ACBUS5                = 0b00100000
ADBUS6 = ACBUS6                = 0b01000000
ADBUS7 = ACBUS7                = 0b10000000
SDA                            = SDA_0 | SDA_1
# fmt: on


msleep = lambda ms: time.sleep(ms / 1000)

class ReadWrite(enum.IntEnum):
    READ = 0x01
    WRITE = 0x00

class SpiMode(enum.IntEnum):
    MODE_0 = enum.auto()
    MODE_1 = enum.auto()
    MODE_2 = enum.auto()
    MODE_3 = enum.auto()


class BitOrder(enum.Enum):
    MSB = enum.auto()
    LSB = enum.auto()


class MPSSE(FTD2XX):

    _device_type: FtdiDevice = None
    _spi_mode: SpiMode = None
    _bit_order: BitOrder = None
    _adbus_direction: int = None
    _acbus_direction: int = None


    def _cache_device_type(self):
        """Cache the device type for later."""
        self._device_type = self.get_device_info().type

    def open(self, index: int):
        if not self.is_connected:
            super().open(index)
            self._cache_device_type()

    def open_ex(self, descriptor: str, flag: OpenExFlag):
        if not self.is_connected:
            super().open_ex(descriptor, flag)
            self._cache_device_type()

    def close(self):
        if self.is_connected:
            self.set_bit_mode(0x00, Bitmodes.FT_BITMODE_RESET)
            self._device_type = None
            self._spi_mode = None
            self._adbus_direction = None
            self._acbus_direction = None
            super().close()

    @staticmethod
    @contextmanager
    def smart_open_enter(*args, **kwargs) -> Generator[MPSSE, None, None]:
        """
        Gives us the ability to connect to an FTDI device using

        with FTDI.smart_open_enter(*args, **kwargs) as device:
            ...
        """
        # TODO Generator is not the correct return type but returning
        # typing.ContextManager[MPSSE] or contextlib.AbstractContextManager[MPSSE]
        # is not working
        device = MPSSE()
        device.smart_open(*args, **kwargs)
        yield device
        device.close()

    def _set_loopback_state(self, enable: bool):
        """
        Set the loopback state.

        Args:
            enable (bool): true to enable loopback. False otherwise.
        """
        TURN_ON_LOOPBACK_CMD = 0x84
        TURN_OFF_LOOPBACK_CMD = 0x85

        loopback_cmd = TURN_ON_LOOPBACK_CMD if enable else TURN_OFF_LOOPBACK_CMD
        cmd = bytearray()
        cmd += loopback_cmd.to_bytes(1, 'little')
        self.write(bytes(cmd))

    def _get_clock_edge_configuration(
        self,
        data_in: bool,
        data_out: bool,
        spi_mode: SpiMode,
        bit_order: BitOrder,
        transfer_type_bits_en: bool = False,
    ) -> int:
        """
        Get the clock edge configuration param.

        Args:
            data_in (bool): true if data is to be clocked in. False otherwise.
            data_out (bool): true if data is to be clocked out. False otherwise.
            spi_mode (spi_mode_e): the spi mode used for the spi protocal.
            bit_order (bit_order_e): the bit order used for the spi protocal.
            transfer_type_bits_en (bool, optional): true to transfer bits. False to transfer bytes. Defaults to False.

        Returns:
            int: The clock edge config param.
        """
        if transfer_type_bits_en:
            CMD_DATA_OUT_POS_EDGE = CMD_DATA_OUT_BITS_POS_EDGE
            CMD_DATA_OUT_NEG_EDGE = CMD_DATA_OUT_BITS_NEG_EDGE
            CMD_DATA_IN_POS_EDGE = CMD_DATA_IN_BITS_POS_EDGE
            CMD_DATA_IN_NEG_EDGE = CMD_DATA_IN_BITS_NEG_EDGE
            CMD_DATA_IN_POS_OUT_NEG_EDGE = CMD_DATA_BITS_IN_POS_OUT_NEG_EDGE
            CMD_DATA_IN_NEG_OUT_POS_EDGE = CMD_DATA_BITS_IN_NEG_OUT_POS_EDGE
        else:
            CMD_DATA_OUT_POS_EDGE = CMD_DATA_OUT_BYTES_POS_EDGE
            CMD_DATA_OUT_NEG_EDGE = CMD_DATA_OUT_BYTES_NEG_EDGE
            CMD_DATA_IN_POS_EDGE = CMD_DATA_IN_BYTES_POS_EDGE
            CMD_DATA_IN_NEG_EDGE = CMD_DATA_IN_BYTES_NEG_EDGE
            CMD_DATA_IN_POS_OUT_NEG_EDGE = (
                CMD_DATA_BYTES_IN_POS_OUT_NEG_EDGE
            )
            CMD_DATA_IN_NEG_OUT_POS_EDGE = (
                CMD_DATA_BYTES_IN_NEG_OUT_POS_EDGE
            )

        if data_in and data_out:
            if spi_mode in [SpiMode.MODE_0, SpiMode.MODE_3]:
                CLOCK_CMD = CMD_DATA_IN_POS_OUT_NEG_EDGE
            else:
                CLOCK_CMD = CMD_DATA_IN_NEG_OUT_POS_EDGE

        elif data_in and not data_out:
            if spi_mode in [SpiMode.MODE_0, SpiMode.MODE_3]:
                CLOCK_CMD = CMD_DATA_IN_POS_EDGE
            else:
                CLOCK_CMD = CMD_DATA_IN_NEG_EDGE
        else:
            if spi_mode in [SpiMode.MODE_0, SpiMode.MODE_3]:
                CLOCK_CMD = CMD_DATA_OUT_POS_EDGE
            else:
                CLOCK_CMD = CMD_DATA_OUT_NEG_EDGE

        if bit_order == BitOrder.LSB:
            CLOCK_CMD |= CMD_DATA_LSB_FIRST

        return CLOCK_CMD

    def _set_clock_divide_state(self, enable: bool):
        """
        Sets the divide by 5 option for the clock.

        Args:
            enable (bool): true to enable. False to disable.
        """
        DISABLE_CLOCK_DIVIDE = 0x8A
        ENABLE_CLOCK_DIVIDE = 0x8B

        clock_divide_cmd = ENABLE_CLOCK_DIVIDE if enable else DISABLE_CLOCK_DIVIDE

        cmd = bytearray()
        cmd += clock_divide_cmd.to_bytes(1, 'little')
        self.write(bytes(cmd))

    def _get_clock_divisor(self, clock_rate_hz: int) -> int:
        """
        Calculate the divisor required to divide down the master clock frq
        to the requested clock rate.

        Args:
            clock_rate_hz (int): the requested clock rate in hertz.

        Returns:
            int: the divisor.
        """
        CLOCK_DIVISOR = (
            lambda master_clk_hz, requested_clk_hz: (master_clk_hz / requested_clk_hz / 2) - 1
        )

        if self.is_low_speed():
            return round(CLOCK_DIVISOR(12e6, clock_rate_hz))

        if self.is_high_speed():
            if clock_rate_hz <= 12e6:
                self._set_clock_divide_state(True)
                return round(CLOCK_DIVISOR(12e6, clock_rate_hz))
            else:
                return round(CLOCK_DIVISOR(60e6, clock_rate_hz))

    def _enable_drive_only_zero(self):
        """Set the state of the drive only zero feature."""
        cmd = bytearray()
        cmd += CMD_ENABLE_DRIVE_ONLY_ZERO.to_bytes(1, "little")
        cmd += (3).to_bytes(2, "little")
        self.write(bytes(cmd))

    def _set_three_phase_clocking(self, enable: bool):
        """
        Set the state of the three phase clocking.

        Args:
            enable (bool): true to enable, false otherwise.
        """
        three_phase_clocking_cmd = CMD_ENABLE_3PHASE_CLOCKING if enable else CMD_DISABLE_3PHASE_CLOCKING
        
        cmd = bytearray()
        cmd += three_phase_clocking_cmd
        self.write(cmd)

    def set_clock_rate(self, clock_rate_hz: int):
        """
        Set the output clock to the requested clock rate.

        Args:
            clock_rate_hz (int): The clock rate in hertz.
        """
        SET_CLOCK_FREQUENCY_CMD = 0x86
        divisor = self._get_clock_divisor(clock_rate_hz)

        cmd = bytearray()
        cmd += SET_CLOCK_FREQUENCY_CMD.to_bytes(1, "little")
        cmd += divisor.to_bytes(1, "little")
        self.write(cmd)

    def _configure_for_mpsse_mode(self, clock_rate_hz: int, latency_timer_ms: int):
        self.reset_device()
        self.purge(True, True)
        self.set_usb_parameters(2**16, 2**16)
        self.set_chars(0, False, 0, False)
        self.set_timeouts(5000, 5000)
        self.set_latency_timer(latency_timer_ms)
        self.set_bit_mode(0x00, Bitmodes.FT_BITMODE_RESET)
        self.set_bit_mode(0x00, Bitmodes.FT_BITMODE_MPSSE)
        self._set_loopback_state(True)
        # SyncMPSSE
        msleep(50)
        self.set_clock_rate(clock_rate_hz)
        msleep(20)
        self._set_loopback_state(False)
        # EmptyDeviceInputBuff

    def get_adbus(self) -> int:
        """
        Get the state of the adbus.

        Returns:
            int: The state of the adbus.
        """
        write_data = bytearray()
        write_data += CMD_GET_DATA_BITS_LOWBYTE.to_bytes(1, "little")
        write_data += CMD_SEND_IMMEDIATE.to_bytes(1, "little")
        self.write(bytes(write_data))

        read_data = self.read(1)
        return read_data[0]

    def get_adbus_bit(self, bitmask) -> bool:
        """
        Get the state of a single bit of the adbus.

        Args:
            bitmask (int): the bitmask.

        Returns:
            bool: true if high. False if low.
        """
        state = self.get_adbus()
        return bool((state & bitmask) == bitmask)

    def set_adbus(self, state: int, direction: int | None = None):
        """
        Set the state of the adbus.

        Not specifying a direction will leave it in its current state.

        NOTE: CLK, MOSI, and MISO will be ignored.

        Args:
            state (int): the state.
            direction (int, optional): the direction. Defaults to None.
        """
        if direction is None:
            direction = self._adbus_direction

        write_data = bytearray()
        write_data += CMD_SET_DATA_BITS_LOWBYTE.to_bytes(1, "little")
        write_data += state.to_bytes(1, "little")
        write_data += direction.to_bytes(1, "little")
        self.write(bytes(write_data))

        self._adbus_direction = direction

    def set_adbus_bit(self, bitmask: int, enable: bool):
        """
        Set a single bit of the adbus high or low.

        Args:
            bitmask (int): the bitmask.
            enable (bool): true to set high, False to set low.
        """
        state = self.get_adbus()

        if enable:
            state |= bitmask
        else:
            state &= bitmask ^ 0xFF

        self.set_adbus(state)

    def get_acbus(self) -> int:
        """
        Get the state of the acbus.

        Returns:
            int: The state of the acbus.
        """
        write_data = bytearray()
        write_data += CMD_GET_DATA_BITS_HIGHBYTE.to_bytes(1, "little")
        write_data += CMD_SEND_IMMEDIATE.to_bytes(1, "little")
        self.write(bytes(write_data))
        
        read_data = self.read(1)
        return read_data[0]

    def get_acbus_bit(self, bitmask) -> bool:
        """
        Get the state of a single bit of the acbus.

        Args:
            bitmask (int): the bitmask.

        Returns:
            bool: true if high. False if low.
        """
        state = self.get_acbus()
        return bool((state & bitmask) == bitmask)

    def set_acbus(self, state: int, direction: int | None = None):
        """
        Set the state of the acbus.

        Not specifying a direction will leave it in its current state.

        Args:
            state (int): the state.
            direction (int, optional): the direction. Defaults to None.
        """
        if direction is None:
            direction = self._acbus_direction

        write_data = bytearray()
        write_data += CMD_SET_DATA_BITS_HIGHBYTE.to_bytes(1, "little")
        write_data += state.to_bytes(1, "little")
        write_data += direction.to_bytes(1, "little")
        self.write(bytes(write_data))

        self._acbus_direction = direction

    def set_acbus_bit(self, bitmask: int, enable: bool):
        """
        Set a single bit of the acbus high or low.

        Args:
            bitmask (int): the bitmask.
            enable (bool): true to set high, False to set low.
        """
        state = self.get_acbus()

        if enable:
            state |= bitmask
        else:
            state &= bitmask ^ 0xFF

        self.set_acbus(state)

    def set_spi_mode(self, spi_mode: SpiMode):
        """
        Set the spi mode for the SPI protocal.

        Args:
            spi_mode (SpiMode): the spi mode.
        """
        self._spi_mode = spi_mode

    def set_spi_bit_order(self, bit_order: BitOrder):
        """
        Set the bit order for the SPI protocal.

        Args:
            bit_oder (BitOrder): the bit order.
        """
        self._bit_order = bit_order

    def configure_for_spi(
        self,
        clock_rate_hz: int,
        latency_timer_ms: int,
        spi_mode: SpiMode,
        bit_order: BitOrder,
        direction: int,
        value: int,
    ):
        """
        Configure the ftdi device for msse spi.

        Args:
            clock_rate_hz (int): the clock rate in hertz.
            latency_timer_ms (int): the latency timer in milliseconds.
            spi_mode (spi_mode_e): the spi mode to use.
            bit_order (bit_order_e): the bit order to use.
            direction (int): the direction of the adbus.
            value (int): the initial value of the adbus.
        """
        self._configure_for_mpsse_mode(clock_rate_hz, latency_timer_ms)
        self.set_adbus(value, direction)

        self.set_spi_mode(spi_mode)
        self.set_spi_bit_order(bit_order)

    def spi_read(
        self, read_count: int, chip_select: int | None = CS, active_low: bool = False
    ) -> bytes:
        """
        Read data from a spi device.

        NOTE: Specifing `None` as the chip select will not use a chip select.

        Args:
            read_count (int): the number of bytes to read.
            chip_select (int, optional): the chip select to use. Defaults to CS.
            active_low (bool, optional): true if the chip select is active low. Defaults to False.

        Returns:
            bytes: the data read from the device.
        """
        if chip_select is not None:
            # Assert the chip select is an output.
            direction = self._adbus_direction
            direction |= chip_select
            self.set_adbus(self.get_adbus(), direction)
            self.set_adbus_bit(chip_select, True & (not active_low))

        clock_config = self._get_clock_edge_configuration(
            False,
            True,
            self._spi_mode,
            self._bit_order,
            False,
        )

        MAX_READ_COUNT = 2**16
        total_read = bytearray()
        for p in range(math.ceil(read_count / MAX_READ_COUNT)):
            count: int = min(MAX_READ_COUNT, read_count - (p * MAX_READ_COUNT))

            data_to_write = bytearray()
            data_to_write += clock_config.to_bytes(1, "little")
            data_to_write += (count - 1).to_bytes(
                2, "little"
            )  # sending 0x0000 requests 1 byte, 0xFFFF requests 65536 bytes
            data_to_write += CMD_SEND_IMMEDIATE.to_bytes(1, "little")

            self.write(bytes(data_to_write))
            read_data = self.read(count)

            total_read += bytes(read_data)

        if chip_select is not None:
            # Assert the chip select is an output.
            direction = self._adbus_direction
            direction |= chip_select
            self.set_adbus(self.get_adbus(), direction)
            self.set_adbus_bit(chip_select, True & (not active_low))

    def spi_write(
        self, data: bytes, chip_select: int | None = CS, active_low: bool = False
    ):
        """
        Write data to a spi device.

        NOTE: Specifing `None` as the chip select will not use a chip select.

        Args:
            data (bytes): the data to write.
            chip_select (int, optional): the chip select to use. Defaults to CS.
            active_low (bool, optional): true if the chip select is active low. Defaults to False.
        """
        if chip_select is not None:
            # Assert the chip select is an output.
            direction = self._adbus_direction
            direction |= chip_select
            self.set_adbus(self.get_adbus(), direction)
            self.set_adbus_bit(chip_select, True & (not active_low))

        clock_config = self._get_clock_edge_configuration(
            True,
            False,
            self._spi_mode,
            self._bit_order,
            False,
        )

        MAX_WRITE_COUNT = 2**16

        for p in range(math.ceil(len(data) / MAX_WRITE_COUNT)):
            count: int = min(MAX_WRITE_COUNT, len(data) - (p * MAX_WRITE_COUNT))
            starting_idx = p * MAX_WRITE_COUNT
            ending_idx = starting_idx + count

            data_to_write = bytearray()
            data_to_write += clock_config.to_bytes(1, "little")
            data_to_write += (count - 1).to_bytes(
                2, "little"
            )  # sending 0x0000 requests 1 byte, 0xFFFF requests 65536 bytes
            data_to_write += CMD_SEND_IMMEDIATE.to_bytes(1, "little")

            self.write(bytes(data_to_write))
            self.write(bytes(data[starting_idx:ending_idx]))

        if chip_select is not None:
            # Assert the chip select is an output.
            direction = self._adbus_direction
            direction |= chip_select
            self.set_adbus(self.get_adbus(), direction)
            self.set_adbus_bit(chip_select, True & (not active_low))

    def spi_write_read(
        self, data: bytes, chip_select: int | None = CS, active_low: bool = False
    ) -> bytes:
        """
        Write and read data simultaneously to a spi device.

        NOTE: Specifing `None` as the chip select will not use a chip select.

        Args:
            data (bytes): the data to write.
            chip_select (int, optional): the chip select to use. Defaults to CS.
            active_low (bool, optional): true if the chip select is active low. Defaults to False.

        Returns:
            bytes: the data read from the device.
        """
        if chip_select is not None:
            # Assert the chip select is an output.
            direction = self._adbus_direction
            direction |= chip_select
            self.set_adbus(self.get_adbus(), direction)
            self.set_adbus_bit(chip_select, True & (not active_low))

        clock_config = self._get_clock_edge_configuration(
            True,
            True,
            self._spi_mode,
            self._bit_order,
            False,
        )

        MAX_WRITE_COUNT = 2**16
        total_read = bytearray()
        for p in range(math.ceil(len(data) / MAX_WRITE_COUNT)):
            count: int = min(MAX_WRITE_COUNT, len(data) - (p * MAX_WRITE_COUNT))
            starting_idx = p * MAX_WRITE_COUNT
            ending_idx = starting_idx + count

            data_to_write = bytearray()
            data_to_write += clock_config.to_bytes(1, "little")
            data_to_write += (count - 1).to_bytes(
                2, "little"
            )  # sending 0x0000 requests 1 byte, 0xFFFF requests 65536 bytes
            data_to_write += CMD_SEND_IMMEDIATE.to_bytes(1, "little")

            self.write(bytes(data_to_write))
            self.write(bytes(data[starting_idx:ending_idx]))
            read_data = self.read(count)

            total_read += bytes(read_data)

        if chip_select is not None:
            # Assert the chip select is an output.
            direction = self._adbus_direction
            direction |= chip_select
            self.set_adbus(self.get_adbus(), direction)
            self.set_adbus_bit(chip_select, True & (not active_low))

        return total_read

    def configure_for_i2c(
        self,
        clock_rate_hz: int,
        latency_timer_ms: int,
        three_phase_clocking: bool,
        drive_only_zero_feature: bool,
    ):
        """
        Configure the ftdi device for msse i2c.

        Args:
            clock_rate_hz (int): the clock rate in hertz.
            latency_timer_ms (int): the latency timer in milliseconds.
            three_phase_clocking (bool): true to enable three phase locking, false otherwise.
            drive_only_zero_feature (bool): true to enable the drive only zero feature, false otherwise.
        """
        if three_phase_clocking:
            clock_rate_hz = clock_rate_hz * 3 / 2

        self._configure_for_mpsse_mode(clock_rate_hz, latency_timer_ms)
        self.set_adbus(0x13, 0x13)
        if (
            self._device_type == FtdiDevice.FT_DEVICE_232H
            and drive_only_zero_feature
        ):
            self._enable_drive_only_zero()
        self._set_three_phase_clocking(three_phase_clocking)

    def check_acks(self, acks: bytes, slave_address: int):
        """
        Check the set of acknowledgment bits.

        Raises:
            I2cNackError: on one of the acknowledgment bits being a NACK.
        """
        for ack in acks:
            if ack != 0x01:
                raise I2cNackError(slave_address)

    def i2c_start_cmd(self) -> bytearray:
        """
        Return the set of bytes required to trigger the start condition
        for the i2c protocal.
        """
        buffer = bytearray()

        # Set SCL (CLK) High and SDA (DATA) Low (Idle state)
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL | SDA, SCL | SDA])

        # Set SCL (CLK) High and SDA (DATA) Low
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL, SCL | SDA])

        # Set SCL (CLK) Low and SDA (DATA) Low
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL ^ 0xFF | SDA ^ 0xFF, SCL | SDA])

        return buffer

    def i2c_write_cmd(self, data: bytes) -> bytearray:
        """
        Return the set of bytes required to write the desired bytes to
        the slave for the i2c protocal.
        """
        buffer = bytearray()

        for byte in data:

            # Command to clock out 1 byte
            buffer += bytes([CMD_DATA_OUT_BYTES_NEG_EDGE, 0x00, 0x00, byte])

            # Set SCL (CLK) Low and SDA (DATA) High
            buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL ^ 0xFF | SDA, SCL | SDA])

            # Command to clock in 1 byte
            buffer += bytes([CMD_DATA_IN_BYTES_POS_EDGE, 0x00, 0x00])

            # SEND_IMMEDIATE

        return buffer

    def i2c_address_cmd(self, slave_address: int, read_write: ReadWrite) -> bytes:
        """
        Return the set of bytes required to read / write to a specific slave
        for the i2c protocal.
        """
        slave_address = slave_address << 1 | read_write

        return self.i2c_write_cmd(slave_address.to_bytes(1, "little"))

    def i2c_read_cmd(self, read_count: int) -> bytearray:
        """
        Return the set of bytes required to read a set of bytes from
        the slave for the i2c protocal.
        """
        buffer = bytearray()

        for _ in range(read_count - 1):
            # Command to clock in 1 byte
            buffer += bytes([CMD_DATA_IN_BYTES_POS_EDGE, 0x00, 0x00])

            # Command to clock out ack
            buffer += bytes([CMD_DATA_OUT_BITS_NEG_EDGE, 0x00, SEND_ACK])

            # Set SCL (CLK) Low and SDA (DATA) High
            buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL ^ 0xFF | SDA, SCL | SDA])

        # Command to clock in 1 byte
        buffer += bytes([CMD_DATA_IN_BYTES_POS_EDGE, 0x00, 0x00])

        # Command to clock out nack
        buffer += bytes([CMD_DATA_OUT_BITS_NEG_EDGE, SEND_NACK])

        # Set SCL (CLK) Low and SDA (DATA) High
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL ^ 0xFF | SDA, SCL | SDA])

        return buffer

    def i2c_stop_cmd(self) -> bytearray:
        """
        Return the set of bytes required to trigger the stop condition
        for the i2c protocal.
        """
        buffer = bytearray()

        # Set SCL (CLK) Low and SDA (DATA) High
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL ^ 0xFF | SDA, SCL | SDA])

        # Set SCL (CLK) Low and SDA (DATA) Low
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL ^ 0xFF | SDA ^ 0xFF, SCL | SDA])

        # Set SCL (CLK) High and SDA (DATA) Low
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL, SCL | SDA])

        # Set SCL (CLK) High and SDA (DATA) Low (Idle state)
        buffer += bytes([CMD_SET_DATA_BITS_LOWBYTE, SCL | SDA, SCL | SDA])

        return buffer

    def i2c_read(self, slave_address: int, read_count: int) -> bytes:
        """
        Read data from the i2c device.

        Args:
            slave_address (int): the slave address.
            read_count (int): number of bytes to read.

        Raises:
            I2cNackError: on receiving a NACK from the slave

        Returns:
            bytes: the data read from the device.
        """
        self.purge(True, True)

        buffer = bytearray()
        buffer += self.i2c_start_cmd()
        buffer += self.i2c_address_cmd(slave_address, ReadWrite.READ)
        buffer += self.i2c_read_cmd(read_count)
        buffer += self.i2c_stop_cmd()
        buffer += bytes([CMD_SEND_IMMEDIATE])

        self.write(bytes(buffer))

        # Read the ack bit from sending the slave address
        ack = self.read(1)
        self.check_acks(ack, slave_address)

        return self.read(read_count)

    def i2c_write(self, slave_address: int, data: bytes):
        """
        Write data to the i2c device.

        Args:
            slave_address (int): the slave address.
            data (bytes): the data to read.

        Raises:
            I2cNackError: on receiving a NACK from the slave.
        """
        self.purge(True, True)

        buffer = bytearray()
        buffer += self.i2c_start_cmd()
        buffer += self.i2c_address_cmd(slave_address, ReadWrite.WRITE)
        buffer += self.i2c_write_cmd(data)
        buffer += self.i2c_stop_cmd()
        buffer += bytes([CMD_SEND_IMMEDIATE])

        # Read the ack bit from sending the slave address
        ack = self.read(1)
        self.check_acks(ack, slave_address)

        # Read the ack bits from sending the data
        acks = self.read(len(data))
        self.check_acks(acks, slave_address)
