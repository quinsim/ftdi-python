"""
See the `ftdichip documentation page <https://ftdichip.com/document/programming-guides/>`_
for the D2XX Programmer's Guide. This guide documents all api calls to the FTD2XX.dll used below.

Please make sure the D2XX Drivers are installed. 
The latest installer / installation guide can be found `here <https://ftdichip.com/drivers/d2xx-drivers/>`_
"""
# Standard library imports
from __future__ import annotations
import sys
import enum
import logging
import dataclasses
from typing import Callable, Generator
from contextlib import suppress, contextmanager

# Third party imports

# Local application imports
import ftdi_python._ftd2xx as __LIBRARY__
from ftdi_python.exceptions import (
    ApiCallFailed,
    InvalidHandle,
    NoDevicesMatchingSearch,
    MultipleDevicesMatchingSearch,
)



class FtdiStatus(enum.IntEnum):
    """
    Available status codes returned by the FTD2XX.dll.
    """

    FT_OK = 0
    FT_INVALID_HANDLE = enum.auto()
    FT_DEVICE_NOT_FOUND = enum.auto()
    FT_DEVICE_NOT_OPENED = enum.auto()
    FT_IO_ERROR = enum.auto()
    FT_INSUFFICIENT_RESOURCES = enum.auto()
    FT_INVALID_PARAMETER = enum.auto()
    FT_INVALID_BAUD_RATE = enum.auto()
    FT_DEVICE_NOT_OPENED_FOR_ERASE = enum.auto()
    FT_DEVICE_NOT_OPENED_FOR_WRITE = enum.auto()
    FT_FAILED_TO_WRITE_DEVICE = enum.auto()
    FT_EEPROM_READ_FAILED = enum.auto()
    FT_EEPROM_WRITE_FAILED = enum.auto()
    FT_EEPROM_ERASE_FAILED = enum.auto()
    FT_EEPROM_NOT_PRESENT = enum.auto()
    FT_EEPROM_NOT_PROGRAMMED = enum.auto()
    FT_INVALID_ARGS = enum.auto()
    FT_NOT_SUPPORTED = enum.auto()
    FT_OTHER_ERROR = enum.auto()


class FtdiDevice(enum.IntEnum):
    """
    FTDI Devices supported by the FTD2XX.dll
    """
    FT_DEVICE_232BM = 0
    FT_DEVICE_232AM = enum.auto()
    FT_DEVICE_100AX = enum.auto()
    FT_DEVICE_UNKNOWN = enum.auto()
    FT_DEVICE_2232C = enum.auto()
    FT_DEVICE_232R = enum.auto()
    FT_DEVICE_2232H = enum.auto()
    FT_DEVICE_4232H = enum.auto()
    FT_DEVICE_232H = enum.auto()
    FT_DEVICE_X_SERIES = enum.auto()


@dataclasses.dataclass
class DeviceListInfoNode:
    """
    Structure that represents a single device.
    """
    flags: int
    type: FtdiDevice
    id: int
    loc_id: int
    serial_number: str
    description: str
    ft_handle: int


class Bitmodes(enum.IntEnum):
    """
    Available bitmodes for ftdi devices.
    """
    FT_BITMODE_RESET = 0x00
    FT_BITMODE_ASYNC_BITBANG = 0x01
    FT_BITMODE_MPSSE = 0x02 # (FT2232, FT2232H, FT4232H and FT232H devices only)
    FT_BITMODE_SYNC_BITBANG = 0x04 # (FT232R, FT245R, FT2232, FT2232H, FT4232H and FT232H devices only)
    FT_BITMODE_MCU_HOST = 0x08 # (FT2232, FT2232H, FT4232H and FT232H devices only)
    FT_BITMODE_FAST_SERIAL = 0x10 # (FT2232, FT2232H, FT4232H and FT232H devices only)
    FT_BITMODE_CBUS_BITBANG = 0x20 # (FT232R and FT232H devices only)
    FT_BITMODE_SYNC_FIFO = 0x40 # (FT2232H and FT232H devices only)


class OpenExFlag(enum.IntEnum):
    """
    Supported flags for the FT_OpenEx command.

    Args:
        FT_OPEN_BY_SERIAL_NUMBER: open a device by its serial number.
        FT_OPEN_BY_DESCRIPTION: open a device by its description.
        FT_OPEN_BY_LOCATION: open a device by its location.
    """

    FT_OPEN_BY_SERIAL_NUMBER = 1
    FT_OPEN_BY_DESCRIPTION = 2
    FT_OPEN_BY_LOCATION = 4


class FlowControlFlag(enum.IntEnum):
    """
    Supported flags for the FT_FlowControl command.
    """

    FT_FLOW_NONE = 0x0000
    FT_FLOW_RTS_CTS = 0x0100
    FT_FLOW_DTR_DSR = 0x0200
    FT_FLOW_XON_XOFF = 0x0400


class WordLength(enum.IntEnum):
    """Supported word lengths for the FT_SetDataCharacteristics command."""

    FT_BITS_8 = 8
    FT_BITS_7 = 7


class StopBits(enum.IntEnum):
    """Supported stop bits for the FT_SetDataCharacteristics command."""

    FT_STOP_BITS_1 = 0
    FT_STOP_BITS_2 = 2


class Parity(enum.IntEnum):
    """Supported parity modes for the FT_SetDataCharacteristics command."""

    FT_PARITY_NONE = 0
    FT_PARITY_ODD = 1
    FT_PARITY_EVEN = 2
    FT_PARITY_MARK = 3
    FT_PARITY_SPACE = 4


@dataclasses.dataclass
class ModemStatus:
    """
    The modem status.

    Args:
        CTS (boo): Clear to Send.
        DSR (boo): Data Set Ready.
        RI (boo): Ring Indicator.
        DCD (boo): Data Carrier Detect.
    """

    CTS: bool
    DSR: bool
    RI: bool
    DCD: bool


@dataclasses.dataclass
class LineStatus:
    """
    The line status.

    Args:
        OE (boo): Overrun Error.
        PE (boo): Parity Error.
        FE (boo): Framing Error.
        BI (boo): Break Interrupt.
    """

    OE: bool
    PE: bool
    FE: bool
    BI: bool


"""
Constants used to represent each of the pins on the ADBUS.

These can be used in conjunction with the GetBitMode cmd
to read the state of any of the pins.
"""
ADBUS0 = 0b00000001
ADBUS1 = 0b00000010
ADBUS2 = 0b00000100
ADBUS3 = 0b00001000
ADBUS4 = 0b00010000
ADBUS5 = 0b00100000
ADBUS6 = 0b01000000
ADBUS7 = 0b10000000


_LOGGER = logging.getLogger(__name__)


class FTD2XX:
    """
    The base FTD2XX.dll python wrapper.

    Only some of the methods have been implemented. If your program
    requires a specific feature, please feel free to add it.

    NOTE: This has been updated for D2XX v1.4.
    """

    def __init__(self):
        self._handle = None

    def _validate_handle(self):
        """
        Validate the handle to the ftdi device.

        Raises:
            InvalidHandle: on the handle being invalid.
        """
        if self._handle is None:
            raise InvalidHandle()

    @property
    def is_connected(self):
        """True if we are connected to an ftdi device. False otherwise."""
        with suppress(InvalidHandle):
            self._validate_handle()
            return True
        return False

    def _get_FtdiStatus(self, status: int) -> FtdiStatus | int:
        """
        Get the ft status given the status id.

        Args:
            status (int): the status id.

        Returns:
            FtdiStatus | int: the ft status if the id is valid else the id.
        """
        try:
            return FtdiStatus(status)
        except ValueError:
            return status

    def _call_ft(self, function: Callable, *args, validate_handle: bool = True):
        """
        Call a function from the FTD2XX.dll.

        Args:
            function (Callable): the function to call.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if validate_handle:
            self._validate_handle()

        status_id = function(*args)
        status = self._get_FtdiStatus(status_id)
        if status != FtdiStatus.FT_OK:
            raise ApiCallFailed(function, args, status)

    def set_vid_pid(self, vid: int, pid: int):
        """
        A command to include a custom VID and PID combination within the internal device list table.
        This will allow the driver to load for the specified VID and PID combination.

        Args:
            vid (int): the device vendor id.
            pid (int): the device product id.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform == "win32":
            raise SystemError(f"{sys.platform} is not supported.")

        self._call_ft(
            __LIBRARY__.FT_SetVIDPID,
            __LIBRARY__.UINT32(vid),
            __LIBRARY__.UINT32(pid),
            validate_handle=False,
        )

    def get_vid_pid(self) -> tuple[int, int]:
        """
        A command to retrieve the current VID and PID combination from within the internal device list table.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            tuple[int, int]: (vendor id, product id).
        """
        if sys.platform == "win32":
            raise SystemError(f"{sys.platform} is not supported.")

        vid = __LIBRARY__.UINT32()
        pid = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_GetVIDPID,
            vid,
            pid,
            validate_handle=False,
        )
        return (vid.value, pid.value)

    def create_device_info_list(self) -> int:
        """
        This function builds a device information list and returns the number of D2XX devices connected to the system.
        The list contains information about both unopen and open devices.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the number of devices connected to the system.
        """
        number_of_devices = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_CreateDeviceInfoList,
            number_of_devices,
            validate_handle=False,
        )
        return number_of_devices.value

    def get_number_of_devices(self) -> int:
        """
        Get the number of devices connected to the pc.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the number of devices connected to the system.
        """
        return self.create_device_info_list()

    def get_device_info_list(self) -> list[DeviceListInfoNode]:
        """
        This function returns a device information list and the number of D2XX devices in the list.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            list[DeviceListInfoNode]: the detected devices' info.
        """
        number_of_devices = __LIBRARY__.UINT32(self.create_device_info_list())
        device_list_type = (
            __LIBRARY__.FT_DEVICE_LIST_INFO_NODE * number_of_devices.value
        )
        device_list = device_list_type()
        # TODO update when __LIBRARY__.FT_GetDeviceInfoList.argtypes
        self._call_ft(
            __LIBRARY__.FT_GetDeviceInfoList,
            device_list,
            number_of_devices,
            validate_handle=False,
        )
        # TODO serial number and description is not working

        return [
            DeviceListInfoNode(
                device_list[i].Flags,
                FtdiDevice(device_list[i].Type),
                device_list[i].ID,
                device_list[i].LocId,
                str(device_list[i].SerialNumber, "utf-8"),
                str(device_list[i].Description, "utf-8"),
                device_list[i].ftHandle,
            )
            for i in range(number_of_devices.value)
        ]

    def get_device_info_detail(self, index: int) -> DeviceListInfoNode:
        """
        This function returns an entry from the device information list.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            DeviceListInfoNode: the requested device's info.
        """
        # FT_GetDeviceInfoDetail
        self.create_device_info_list()

        flags = __LIBRARY__.UINT32()
        type = __LIBRARY__.UINT32()
        id = __LIBRARY__.UINT32()
        loc_id = __LIBRARY__.UINT32()
        serial_number = __LIBRARY__.STRING(16)()
        description = __LIBRARY__.STRING(64)()
        ft_handle = __LIBRARY__.FT_HANDLE()

        self._call_ft(
            __LIBRARY__.FT_GetDeviceInfoDetail,
            __LIBRARY__.UINT32(index),
            flags,
            type,
            id,
            loc_id,
            serial_number,
            description,
            ft_handle,
            validate_handle=False,
        )

        return DeviceListInfoNode(
            flags.value,
            FtdiDevice(type.value),
            id.value,
            loc_id.value,
            str(serial_number.value, "utf-8"),
            str(description.value, "utf-8"),
            ft_handle.value,
        )

    # TODO
    # def list_devices(self):
    #     """

    #     Raises:
    #         ApiCallFailed: on the api call to FTD2XX.dll failing.
    #     """
    #     FT_LIST_NUMBER_ONLY = 0x80000000
    #     FT_LIST_BY_INDEX = 0x40000000
    #     FT_LIST_ALL = 0x20000000

    #     FT_ListDevices
    #     pass

    def open(self, index: int):
        """
        Open the device and return a handle which will be used for subsequent accesses.

        Args:
            index (int): the index of the device to open (0-based).

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if not self.is_connected:
            handle = __LIBRARY__.FT_HANDLE()
            self._call_ft(
                __LIBRARY__.FT_Open,
                index,
                handle,
                validate_handle=False,
            )
            self._handle = handle

    def open_ex(self, descriptor: str, flag: OpenExFlag):
        """
        Open the specified device and return a handle that will be used for subsequent accesses.
        The device can be specified by its serial number, device description or location.

        This function can also be used to open multiple devices simultaneously. Multiple devices can be specified
        by serial number, device description or location ID (location information derived from the physical
        location of a device on USB). Location IDs for specific USB ports can be obtained using the utility
        USBView and are given in hexadecimal format. Location IDs for devices connected to a system can be
        obtained by calling FT_GetDeviceInfoList or FT_ListDevices with the appropriate flags.

        Args:
            descriptor (str): the descriptor used when opening the device. Depends on the `flag`.
            flag (OpenExFlag): the type of descriptor being used when opening the device.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if not self.is_connected:
            handle = __LIBRARY__.FT_HANDLE()
            self._call_ft(
                __LIBRARY__.FT_OpenEx,
                descriptor,
                __LIBRARY__.UINT32(flag),
                handle,
                validate_handle=False,
            )
            self._handle = handle

    def open_by_serial_number(self, serial_number: str):
        """
        Open a device by its serial number.

        Args:
            serial_number (str): the serial number.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self.open_ex(bytes(serial_number, "utf-8"), OpenExFlag.FT_OPEN_BY_SERIAL_NUMBER)

    def open_by_description(self, description: str):
        """
        Open a device by its description.

        Args:
            description (str): the description.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self.open_ex(bytes(description, "utf-8"), OpenExFlag.FT_OPEN_BY_DESCRIPTION)

    def open_by_location(self, location: str):
        """
        Open a device by its location.

        Args:
            location (str): the location.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self.open_ex(location, OpenExFlag.FT_OPEN_BY_LOCATION)

    def get_matching_devices(
        self,
        type: FtdiDevice = None,
        id: int = None,
        locId: int = None,
        serial_number: str = None,
        description: str = None,
    ) -> list[DeviceListInfoNode]:
        """
        Gets a list of devices that match your search parameters.

        Args:
            type (FtdiDevice, optional): the FTDI Device type. Defaults to None.
            id (int, optional): the FTDI id. Defaults to None.
            locId (int, optional): the FTDI location id. Defaults to None.
            serial_number (str, optional): the FTDI serial number. Defaults to None.
            description (str, optional): the FTDI description. Defaults to None.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            list[DeviceListInfoNode]: a list of FTDI devices matching your search parameters.
        """
        device_list = self.get_device_info_list()

        return [
            device
            for device in device_list
            if (type is None or type == device.type)
            and (id is None or id == device.id)
            and (locId is None or locId == device.loc_id)
            and (serial_number is None or serial_number == device.serial_number)
            and (description is None or description == device.description)
        ]

    def smart_open(
        self,
        index: int = None,
        type: FtdiDevice = None,
        id: int = None,
        location: int = None,
        serial_number: str = None,
        description: str = None,
    ):
        """
        Opens the FTDI device with the specified device information.

        You can specify any combination of the optional perameters
        when attempting to connect to an FTDI chip. The device list
        will be searched for any device matching your specifications.

        NOTE: index takes priority and will cause all other args to be ignored.

        Args:
            index (int, optional): the index in the device list.
            type (FtdiDevice, optional): FTDI Type. Defaults to None.
            id (int, optional): the FTDI device ID. Defaults to None.
            location (int, optional): the location of the FTDI device. Defaults to None.
            serial_number (str, optional): the serial number of the FTDI device. Defaults to None.
            description (str, optional): the description of the FTDI device. Defaults to None.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
            NoDevicesMatchingSearch: on no devices matching the search parameters.
            MultipleDevicesMatchingSearch: on multiple devices matching the search parameters.
        """
        if index is not None:
            self.open(index)
        else:
            user_specifications = [type, id, location, serial_number, description]
            device_list = self.get_matching_devices(*user_specifications)

            device = DeviceListInfoNode(
                None, type, id, location, serial_number, description, None
            )

            if not len(device_list):
                raise NoDevicesMatchingSearch(device)

            if len(device_list) > 1:
                raise MultipleDevicesMatchingSearch(device, device_list)

            self.open_by_serial_number(device_list[0].serial_number)

    def close(self):
        """
        Close an open device.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if self.is_connected:
            self._call_ft(__LIBRARY__.FT_Close, self._handle)
            self._handle = None

    @staticmethod
    @contextmanager
    def smart_open_enter(*args, **kwargs) -> Generator[FTD2XX, None, None]:
        """
        Gives us the ability to connect to an FTDI device using

        with FTDI.smart_open_enter(*args, **kwargs) as device:
            ...

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        # TODO Generator is not the correct return type but returning
        # typing.ContextManager[FTD2XX] or contextlib.AbstractContextManager[FTD2XX]
        # is not working
        device = FTD2XX()
        device.smart_open(*args, **kwargs)
        yield device
        device.close()

    def read(self, read_count: int) -> bytes:
        """
        Read data from the device.

        Args:
            read_count (int): the number of bytes to read.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            bytes: the bytes read.
        """
        number_of_bytes_read = __LIBRARY__.UINT32()
        bytes_read = __LIBRARY__.STRING(read_count)()
        self._call_ft(
            __LIBRARY__.FT_Read,
            self._handle,
            bytes_read,
            read_count,
            number_of_bytes_read,
        )

        if number_of_bytes_read.value != read_count:
            raise Exception(
                f"Only {number_of_bytes_read.value} / {read_count} bytes were read."
            )

        return bytes(bytes_read.raw)

    def write(self, data: bytes):
        """
        Write data to the device.

        Args:
            data (bytes): the data to write.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        number_of_bytes_written = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_Write,
            self._handle,
            data,
            __LIBRARY__.UINT32(len(data)),
            number_of_bytes_written,
        )

        if number_of_bytes_written.value != len(data):
            raise Exception(
                f"Only {number_of_bytes_written.value} / {len(data)} bytes were written."
            )

    def set_baudrate(self, baudrate_hz: int):
        """
        This function sets the baudrate for the device.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetBaudRate, self._handle, __LIBRARY__.UINT32(baudrate_hz)
        )

    def set_divisor(self, baudrate_hz: int):
        """
        This function sets the baudrate for the device. 
        It is used to set non-standard baudrates.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetDivisor, self._handle, __LIBRARY__.UINT16(baudrate_hz)
        )

    def set_data_characteristics(
        self, word_length: WordLength, stop_bits: StopBits, parity: Parity
    ):
        """
        This function sets the data characteristics for the device.

        Args:
            word_length (WordLength): the number of bits per word.
            stop_bits (StopBits): the number of stop bits.
            parity (Parity): parity mode when validating communication.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetDataCharacteristics,
            self._handle,
            __LIBRARY__.UINT8(word_length),
            __LIBRARY__.UINT8(stop_bits),
            __LIBRARY__.UINT8(parity),
        )

    def set_timeouts(self, read_timeout_ms: int, write_timeout_ms: int):
        """
        This function sets the read and write timeouts for the device.

        Args:
            read_timeout_ms (int): the read timeout in milliseconds.
            write_timeout_ms (int): the write timeout in milliseconds.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetTimeouts,
            self._handle,
            __LIBRARY__.UINT32(read_timeout_ms),
            __LIBRARY__.UINT32(write_timeout_ms),
        )

    def set_flow_control(
        self,
        flag: FlowControlFlag,
        xon: int = -1,
        xoff: int = -1,
    ):
        """
        This function sets the flow control for the device.

        NOTE: xon xoff will be ignored unless in FT_FLOW_XON_XOFF mode.

        Args:
            flag (FlowControlFlag): the type of flow control.
            xon (int, optional): character used to signal Xon. Defaults to -1.
            xoff (int, optional): character used to signal Xoff. Defaults to -1.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if flag == FlowControlFlag.FT_FLOW_XON_XOFF:
            assert (
                xon != -1 and xoff != -1
            ), f"xon and xoff must be specified when using {FlowControlFlag.FT_FLOW_XON_XOFF}"

        self._call_ft(
            __LIBRARY__.FT_SetFlowControl,
            self._handle,
            __LIBRARY__.UINT16(flag),
            __LIBRARY__.UINT8(xon),
            __LIBRARY__.UINT8(xoff),
        )

    def set_dtr(self):
        """
        This function sets the Data Terminal Ready (DTR) control signal.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetDtr,
            self._handle,
        )

    def clr_dtr(self):
        """
        This function clears the Data Terminal Ready (DTR) control signal.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_ClrDtr,
            self._handle,
        )

    def set_rts(self):
        """
        This function sets the Request To Send (RTS) control signal.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetRts,
            self._handle,
        )

    def clr_rts(self):
        """
        This function clears the Request To Send (RTS) control signal.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_ClrRts,
            self._handle,
        )

    def get_modem_status(self) -> tuple[ModemStatus, LineStatus]:
        """
        Gets the modem status and line status from the device

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            tuple[ModemStatus, LineStatus]: (the modem status, the line status)
        """
        CTS = 0x10
        DSR = 0x20
        RI = 0x40
        DCD = 0x80

        OE = 0x02
        PE = 0x04
        FE = 0x08
        BI = 0x10

        if sys.platform != "win32":
            _LOGGER.warning(f"The line status is not supported by {sys.platform}.")

        status = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_GetModemStatus,
            self._handle,
            status,
        )

        status = status.value
        modem_status = ModemStatus(
            bool(CTS & status),
            bool(DSR & status),
            bool(RI & status),
            bool(DCD & status),
        )

        line_status = LineStatus(
            bool(OE & (status >> 8)),
            bool(PE & (status >> 8)),
            bool(FE & (status >> 8)),
            bool(BI & (status >> 8)),
        )
        return modem_status, line_status

    def get_queue_status(self) -> int:
        """
        Gets the number of bytes in the receive queue.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the number of bytes in the receive queue.
        """
        rx_count = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_GetQueueStatus,
            self._handle,
            rx_count,
        )
        return rx_count.value

    def get_device_info(self) -> DeviceListInfoNode:
        """
        Get device information for an open device.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            DeviceListInfoNode: the info from the connected device.
        """
        type = __LIBRARY__.UINT32()
        id = __LIBRARY__.UINT32()
        serial_number = __LIBRARY__.STRING(16)()
        description = __LIBRARY__.STRING(64)()

        self._call_ft(
            __LIBRARY__.FT_GetDeviceInfo,
            self._handle,
            type,
            id,
            serial_number,
            description,
            None,
        )
        return DeviceListInfoNode(
            1,
            FtdiDevice(type.value),
            id.value,
            None,
            str(serial_number.value, "utf-8"),
            str(description.value, "utf-8"),
            None,
        )

    def get_driver_version(self) -> str:
        """
        This function returns the D2XX driver version number.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            str: the driver version in the form major.minor.build
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        version = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_GetDriverVersion,
            self._handle,
            version,
        )

        version_str = f"{version.value:0X}"
        major = version_str[0]
        minor = version_str[1:3]
        build = version_str[3:5]
        return f"{major}.{minor}.{build}"

    def get_library_version(self) -> str:
        """
        This function returns D2XX DLL version number.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            str: the library version in the form major.minor.build
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        FT_DRIVER_TYPE_D2XX = 0
        FT_DRIVER_TYPE_VCP = 1

        version = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_GetLibraryVersion,
            version,
            validate_handle=False,
        )
        version_str = f"{version.value:0X}"
        major = version_str[0]
        minor = version_str[1:3]
        build = version_str[3:5]
        return f"{major}.{minor}.{build}"

    def get_com_port_number(self) -> int:
        """
        Retrieves the COM port associated with a device.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the com port.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        com_port = __LIBRARY__.INT32()
        self._call_ft(
            __LIBRARY__.FT_GetComPortNumber,
            self._handle,
            com_port,
        )
        return com_port.value

    def get_status(self) -> tuple[int, int, int]:
        """
        Gets the device status including number of characters in the receive queue,
        number of characters in the transmit queue, and the current event status.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            tuple[int, int, int]: (rx queue count, tx queue count, event status)
        """
        rx_queue_count = __LIBRARY__.UINT32()
        tx_queue_count = __LIBRARY__.UINT32()
        event_status = __LIBRARY__.UINT32()

        self._call_ft(
            __LIBRARY__.FT_GetStatus,
            self._handle,
            rx_queue_count,
            tx_queue_count,
            event_status,
        )
        return (rx_queue_count.value, tx_queue_count.value, event_status.value)

    def set_event_notificiation(
        self,
        rxchar: bool,
        modem_status: bool,
        line_status: bool,
    ):
        """
        Sets conditions for event notification.

        Args:
            rxchar (bool): trigger an event when a character is received by the device.
            modem_status (bool): trigger an event when a change in the  modem signals has been detected.
            line_status (bool): trigger an event when a change in the line status has been detected.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        FT_EVENT_RXCHAR = 1
        FT_EVENT_MODEM_STATUS = 2
        FT_EVENT_LINE_STATUS = 4
        flag = (
            (FT_EVENT_RXCHAR if rxchar else 0)
            | (FT_EVENT_MODEM_STATUS if modem_status else 0)
            | (FT_EVENT_LINE_STATUS if line_status else 0)
        )

        event_handle = None  # TODO
        self._call_ft(
            __LIBRARY__.FT_SetEventNotification,
            self._handle,
            __LIBRARY__.UINT32(flag),
            __LIBRARY__.FT_HANDLE(event_handle),
        )

        return event_handle

    def set_chars(
        self,
        event_character: int,
        event_character_en: bool,
        error_cheracter: int,
        error_character_en: bool,
    ):
        """
        This function sets the special characters for the device.

        Args:
            event_character (int): the event character.
            event_character_en (bool): False if event character is disabled, else True
            error_cheracter (int): the error character
            error_character_en (bool): False if error character is disabled, else True

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetChars,
            self._handle,
            __LIBRARY__.UINT8(event_character),
            __LIBRARY__.UINT8(int(event_character_en)),
            __LIBRARY__.UINT8(error_cheracter),
            __LIBRARY__.UINT8(int(error_character_en)),
        )

    def set_break_on(self):
        """
        Sets the BREAK condition for the device.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetBreakOn,
            self._handle,
        )

    def set_break_off(self):
        """
        Resets the BREAK condition for the device.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetBreakOff,
            self._handle,
        )

    def purge(self, rx: bool, tx: bool):
        """
        This function purges receive and transmit buffers in the device

        Args:
            rx (bool): True to purge rx. False otherwise.
            tx (bool): True to purge tx. False otherwise.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        FT_PURGE_RX = 1
        FT_PURGE_TX = 2

        mask = (FT_PURGE_RX if rx else 0x00) | (FT_PURGE_TX if tx else 0x00)
        self._call_ft(
            __LIBRARY__.FT_Purge,
            self._handle,
            __LIBRARY__.UINT32(mask),
        )

    def reset_device(self):
        """
        This function sends a reset command to the device.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(__LIBRARY__.FT_ResetDevice, self._handle)

    def reset_port(self):
        """
        Send a reset command to the por

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        self._call_ft(__LIBRARY__.FT_ResetPort, self._handle)

    def cycle_port(self):
        """
        Send a cycle command to the USB port.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        self._call_ft(__LIBRARY__.FT_CyclePort, self._handle)

    def rescan(self):
        """
        This function can be of use when trying to recover devices programatically.

        Raises:
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        self._call_ft(__LIBRARY__.FT_Rescan)

    def reload(self, vid: int, pid: int):
        """
        This function forces a reload of the driver for devices with a specific VID and PID combination.

        Args:
            vid (int): the vendor id of the devices to reload the driver for.
            pid (int): the product id of the devices to reload the driver for.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        self._call_ft(
            __LIBRARY__.FT_Reload, __LIBRARY__.UINT16(vid), __LIBRARY__.UINT16(pid)
        )

    def set_reset_pipe_retry_count(self, count: int):
        """
        Set the ResetPipeRetryCount value.

        Args:
            count (int): the required rest pipe retry count.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")

        self._call_ft(
            __LIBRARY__.FT_SetResetPipeRetryCount,
            self._handle,
            __LIBRARY__.UINT16(count),
        )

    def stop_in_task(self):
        """
        Stops the driver's IN task.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(__LIBRARY__.FT_StopInTask, self._handle)

    def restart_in_task(self):
        """
        Restart the driver's IN task.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(__LIBRARY__.FT_RestartInTask, self._handle)

    def set_deadman_timeout(self, timeout_ms: int = 5000):
        """
        This function allows the maximum time in milliseconds that a USB
        request can remain outstanding to be set.

        Args:
            timeout_ms (int, optional): deadman timeout in milliseconds. Defaults to 5000 ms.
        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetDeadmanTimeout,
            self._handle,
            __LIBRARY__.UINT32(timeout_ms),
        )

    # def io_ctl(self):
    #     """Undocumented Function."""

    # def set_wait_mask(self):
    #     """Undocumented Function."""

    # def wait_on_mask(self):
    #     """Undocumented Function."""

    def read_ee(self, offset: int) -> int:
        """
        Read a value from an EEPROM location.

        Args:
            offset (int): the word offset.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the value at that location in the eeprom.
        """
        value = __LIBRARY__.UINT16()
        self._call_ft(
            __LIBRARY__.FT_ReadEE,
            self._handle,
            __LIBRARY__.UINT32(offset),
            value,
        )

        return value.value

    def write_ee(self, offset: int, value: int):
        """
        Write a value to an EEPROM location.

        Args:
            offset (int): the word offset.
            value (int): the value to write to that location.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_WriteEE,
            self._handle,
            __LIBRARY__.UINT32(offset),
            __LIBRARY__.UINT16(value),
        )

    def erase_ee(self):
        """
        Erases the device EEPROM.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(__LIBRARY__.FT_EraseEE, self._handle)

    def ee_read(self) -> __LIBRARY__.FT_PROGRAM_DATA_STRUCTURE:
        """
        Read the contents of the EEPROM.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            __LIBRARY__.FT_PROGRAM_DATA_STRUCTURE: The eeprom data.
        """
        data = __LIBRARY__.FT_PROGRAM_DATA_STRUCTURE()
        self._call_ft(
            __LIBRARY__.FT_EE_Read,
            self._handle,
            data,
        )

        return data

    def ee_read_ex(self):
        """
        Read the contents of the EEPROM and pass strings separately.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            __LIBRARY__.FT_PROGRAM_DATA_STRUCTURE: The eeprom data.
        """
        # TODO
        __LIBRARY__.FT_EE_ReadEx
        raise NotImplementedError()

    def ee_program(self, data: __LIBRARY__.FT_PROGRAM_DATA_STRUCTURE):
        """
        Program the EEPROM.

        Args:
            data (__LIBRARY__.FT_PROGRAM_DATA_STRUCTURE): the data to program.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_EE_Program,
            self._handle,
            data,
        )

    def ee_program_ex(self):
        """
        Program the EEPROM and pass strings separately.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        # TODO
        __LIBRARY__.FT_EE_ProgramEx
        raise NotImplementedError()

    def ee_ua_size(self) -> int:
        """
        Get the available size of the EEPROM user area

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the available size.
        """
        size = __LIBRARY__.UINT32()
        self._call_ft(
            __LIBRARY__.FT_EE_UASize,
            self._handle,
            size,
        )

    def ee_ua_read(self):
        """
        Read the contents of the EEPROM user area.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        # TODO
        __LIBRARY__.FT_EE_UARead
        raise NotImplementedError()

    def ee_ua_write(self):
        """
        Write data into the EEPROM user area.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        # TODO
        __LIBRARY__.FT_EE_UAWrite
        raise NotImplementedError()

    # TODO
    def eeprom_read(self):
        """
        Read data from the EEPROM, this command will work for all
        existing FTDI chipset, and must be used for the FT-X series.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")
        __LIBRARY__.FT_EEPROM_Read
        raise NotImplementedError()

    # TODO
    def eeprom_program(self):
        """
        Write data into the EEPROM, this command will work for all
        existing FTDI chipset,and must be used for the FT-X series.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        if sys.platform != "win32":
            raise SystemError(f"{sys.platform} is not supported")
        __LIBRARY__.FT_EEPROM_Program
        raise NotImplementedError()

    def set_latency_timer(self, timer_ms: int):
        """
        Set the latency timer value.

        Args:
            timer_ms (int): the timer in milliseconds.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetLatencyTimer,
            self._handle,
            __LIBRARY__.UINT8(timer_ms),
        )

    def get_latency_timer(self):
        """
        Get the current value of the latency timer.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the latency timer in milliseconds.
        """
        timer_ms = __LIBRARY__.UINT8()
        self._call_ft(
            __LIBRARY__.FT_GetLatencyTimer,
            self._handle,
            timer_ms,
        )

        return timer_ms.value

    def set_bit_mode(self, direction: int, mode: Bitmodes):
        """
        Enables different chip modes.

        Args:
            direction (int): the direction of the bits.
            mode (Bitmodes): the bitmode to set.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetBitMode,
            self._handle,
            __LIBRARY__.UINT8(direction),
            __LIBRARY__.UINT8(mode),
        )

    def get_bit_mode(self) -> int:
        """
        Gets the instantaneous value of the data bus.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.

        Returns:
            int: the value of the ADUS.
        """
        mode = __LIBRARY__.UINT8()
        self._call_ft(
            __LIBRARY__.FT_GetBitMode, self._handle, mode
        )

        return mode.value

    def set_usb_parameters(
        self,
        in_transfer_size: int,
        out_transfer_size: int,
    ):
        """
        Set the USB request transfer size

        Args:
            in_transfer_size (int): the transfer size for USB in request.
            out_transfer_size (int): the transfer size for USB out request.

        Raises:
            InvalidHandle: on the handle being invalid.
            ApiCallFailed: on the api call to FTD2XX.dll failing.
        """
        self._call_ft(
            __LIBRARY__.FT_SetUSBParameters,
            self._handle,
            __LIBRARY__.UINT32(in_transfer_size),
            __LIBRARY__.UINT32(out_transfer_size),
        )

    # TODO FT_W32_CreateFile
    # TODO FT_W32_CloseHandle
    # TODO FT_W32_ReadFile
    # TODO FT_W32_WriteFile
    # TODO FT_W32_GetOverlappedResult
    # TODO FT_W32_EscapeCommFunction
    # TODO FT_W32_GetCommModemStatus
    # TODO FT_W32_SetupComm
    # TODO FT_W32_SetCommState
    # TODO FT_W32_GetCommState
    # TODO FT_W32_SetCommTimeouts
    # TODO FT_W32_GetCommTimeouts
    # TODO FT_W32_SetCommBreak
    # TODO FT_W32_ClearCommBreak
    # TODO FT_W32_SetCommMask
    # TODO FT_W32_GetCommMask windows only
    # TODO FT_W32_WaitCommEvent
    # TODO FT_W32_PurgeComm
    # TODO FT_W32_GetLastError
    # TODO FT_W32_ClearCommError

    @property
    def has_sync_bitbang(self) -> bool:
        """
        Check whether the connceted device supports Synchronous Bitbang mode.

        Returns:
            bool: true if the ftdi device supports mpsse, false otherwise.
        """
        # FT232R, FT245R, FT2232, FT2232H, FT4232H and FT232H devices only
        self._validate_handle()
        device_type = self.get_device_info().type

        # TODO what about FT245R
        return device_type in (
            FtdiDevice.FT_DEVICE_232R,
            FtdiDevice.FT_DEVICE_2232C,
            FtdiDevice.FT_DEVICE_2232H,
            FtdiDevice.FT_DEVICE_4232H,
            FtdiDevice.FT_DEVICE_232H,
        )

    @property
    def has_mpsse(self) -> bool:
        """
        Check whether the connceted device supports MPSSE mode.

        (I2C, SPI, JTAG, ...)

        Returns:
            bool: true if the ftdi device supports mpsse, false otherwise.
        """
        # FT2232, FT2232H, FT4232H and FT232H devices only
        self._validate_handle()
        device_type = self.get_device_info().type

        return device_type in (
            FtdiDevice.FT_DEVICE_2232C,
            FtdiDevice.FT_DEVICE_2232H,
            FtdiDevice.FT_DEVICE_4232H,
            FtdiDevice.FT_DEVICE_232H,
        )

    @property
    def has_cbus_bitbang(self) -> bool:
        """
        Check whether the connected device supports CBUS BITBANG mode.

        Returns:
            bool: true if the ftdi device supports cbus bitbang mode, false otherwise.
        """
        # FT232R and FT232H devices only
        self._validate_handle()
        device_type = self.get_device_info().type

        return device_type in (
            FtdiDevice.FT_DEVICE_232R,
            FtdiDevice.FT_DEVICE_232H,
        )

    @property
    def is_low_speed(self) -> bool:
        """
        Check whether the connected device is a slow usb-uart bridge.

        Returns:
            bool: true if the ftdi device is a slow usb uart bridge, false otherwise.
        """
        self._validate_handle()
        device_type = self.get_device_info().type
        
        return device_type in (
            FtdiDevice.FT_DEVICE_2232C,
        )

    @property
    def is_high_speed(self) -> bool:
        """
        Check whether the connected device is a high end usb-uart bridge.

        Returns:
            bool: true if the ftdi device is a high end usb uart bridge, false otherwise.
        """
        self._validate_handle()
        device_type = self.get_device_info().type
        
        return device_type in (
            FtdiDevice.FT_DEVICE_2232H,
            FtdiDevice.FT_DEVICE_4232H,
            FtdiDevice.FT_DEVICE_232H,
        )

    def initialize_bitbang(
        self,
        direction: int = 0x00,
        value: int = 0x00,
        latency_timer_ms: int = 16,
        baudrate_hz: int = 1000000,
        sync: bool = False,
    ):
        """
        Initialize the connected ftdi device for bitbang mode.

        Args:
            direction (int, optional): the direction to set the ADBUS to.
            value (int, optional): the initial state of the bus. All pins default to low.
            latency_timer_ms (int, optional): the time in ms to set the latency timer to.
            baudrate_hz (int, optional): the baudrate frequency in hertz.
            sync (bool, optional): whether to use async or sync bitbang. Defaults to async.
        """
        # Default latency timer should by 16ms
        # https://www.ftdichip.com/Support/Knowledgebase/index.html?settingacustomdefaultlaten.htm            
        self._validate_handle()

        if sync:
            assert self.has_sync_bitbang(), f"{self.get_device_info().type} does not support Synchronous Bitbang Mode."

        self.set_latency_timer(latency_timer_ms)
        self.set_bit_mode(0x00, Bitmodes.FT_BITMODE_RESET)
        self.purge()
        self.set_bit_mode(direction, Bitmodes.FT_BITMODE_SYNC_BITBANG if sync else Bitmodes.FT_BITMODE_ASYNC_BITBANG)
        self.set_baudrate(baudrate_hz)
        self.write(value.to_bytes(1, 'little'))

    def initialize_cbus_bitbang(
        self,
        direction: int = 0x0,
        value: int = 0x0,
        latency_timer_ms: int = 16,
        baudrate_hz: int = 1000000,
    ):
        """
        Initialize the connected ftdi device for cbus bitbang mode.

        Args:
            direction (int, optional): the direction to set the ACBUS to.
            value (int, optional): the initial state of the bus. All pins default to low.
            latency_timer_ms (int, optional): the time in ms to set the latency timer to.
            baudrate_hz (int, optional): the baudrate frequency in hertz.
        """
        # Default latency timer should by 16ms
        # https://www.ftdichip.com/Support/Knowledgebase/index.html?settingacustomdefaultlaten.htm
        direction_and_value = (direction & 0xF0) | (value & 0x0F)

        self._validate_handle()

        assert self.has_cbus_bitbang(), f"{self.get_device_info().type} does not support Cbus Bitbang Mode."
        
        self.set_latency_timer(latency_timer_ms)
        self.set_bit_mode(0x00, Bitmodes.FT_BITMODE_RESET)
        self.purge()
        self.set_bit_mode(direction_and_value, Bitmodes.FT_BITMODE_ASYNC_BITBANG)
        self.set_baudrate(baudrate_hz)
        self.write(value.to_bytes(1, 'little'))
