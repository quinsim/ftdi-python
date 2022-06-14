# Standard library imports
from __future__ import annotations

# Third party imports
import pytest

# Local application imports
from ftdi_python.ftd2xx import FTD2XX


@pytest.fixture
def ftdi() -> FTD2XX:
    ftdi = FTD2XX()
    yield ftdi


@pytest.fixture
def connected_ftdi() -> FTD2XX:
    with FTD2XX().smart_open_enter() as ftdi:
        yield ftdi


class TestFTD2XX:

    # def test set_vid_pid(self)
    # def test get_vid_pid(self)

    def test_create_device_info_list(self, ftdi: FTD2XX):
        return_type = ftdi.create_device_info_list()
        assert isinstance(return_type, int)

    def test_get_number_of_devices(self, ftdi: FTD2XX):
        return_type = ftdi.get_number_of_devices()
        assert isinstance(return_type, int)

    def test_get_device_info_list(self, ftdi: FTD2XX):
        from ftdi_python.ftd2xx import DeviceListInfoNode

        return_type = ftdi.get_device_info_list()
        assert isinstance(return_type, list)
        for item in return_type:
            assert isinstance(item, DeviceListInfoNode)

    def test_get_device_info_detail(self, ftdi: FTD2XX):
        from ftdi_python.ftd2xx import DeviceListInfoNode

        number_of_devices = ftdi.get_number_of_devices()
        for index in range(number_of_devices):
            return_type = ftdi.get_device_info_detail(index)
            assert isinstance(return_type, DeviceListInfoNode)

    def test_open_and_close(self, ftdi: FTD2XX):
        number_of_devices = ftdi.get_number_of_devices()
        for index in range(number_of_devices):
            ftdi.open(index)
            assert ftdi.is_connected

            ftdi.close()
            assert not ftdi.is_connected

    def test_open_by_serial_number(self, ftdi: FTD2XX):
        devices = ftdi.get_device_info_list()
        for device in devices:
            ftdi.open_by_serial_number(device.serial_number)
            assert ftdi.is_connected

            ftdi.close()
            assert not ftdi.is_connected

    def test_open_by_description(self, ftdi: FTD2XX):
        devices = ftdi.get_device_info_list()
        for device in devices:
            ftdi.open_by_description(device.description)
            assert ftdi.is_connected

            ftdi.close()
            assert not ftdi.is_connected

    def test_open_by_location(self, ftdi: FTD2XX):
        devices = ftdi.get_device_info_list()
        for device in devices:
            ftdi.open_by_location(device.loc_id)
            assert ftdi.is_connected

            ftdi.close()
            assert not ftdi.is_connected

    # def test_get_matching_devices(self)
    # def test_smart_open(self)
    # def test_smart_open_enter(self)
    # def test_read(self)
    # def test_write(self)

    def test_set_baud_rate(self, connected_ftdi: FTD2XX):
        connected_ftdi.set_baudrate(9600)

    def test_set_divisor(self, connected_ftdi: FTD2XX):
        connected_ftdi.set_divisor(9600)

    def test_set_data_characteristics(self, connected_ftdi: FTD2XX):
        import itertools
        from ftdi_python.ftd2xx import (
            Parity,
            StopBits,
            WordLength,
        )

        for word_length, stop_bits, parity in itertools.product(
            WordLength, StopBits, Parity
        ):
            connected_ftdi.set_data_characteristics(word_length, stop_bits, parity)

    def test_set_timeouts(self, connected_ftdi: FTD2XX):
        connected_ftdi.set_timeouts(0, 0)

    def test_set_flow_control(self, connected_ftdi: FTD2XX):
        from ftdi_python.ftd2xx import FlowControlFlag

        for flag in FlowControlFlag:
            connected_ftdi.set_flow_control(flag, 0, 0)

    def test_set_and_clear_dtr(self, connected_ftdi: FTD2XX):
        connected_ftdi.set_dtr()
        connected_ftdi.clr_dtr()

    def test_set_and_clear_rts(self, connected_ftdi: FTD2XX):
        connected_ftdi.set_rts()
        connected_ftdi.clr_rts()

    def test_get_modem_status(self, connected_ftdi: FTD2XX):
        from ftdi_python.ftd2xx import LineStatus, ModemStatus

        modem, line = connected_ftdi.get_modem_status()
        assert isinstance(modem, ModemStatus)
        assert isinstance(line, LineStatus)

    def test_get_queue_status(self, connected_ftdi: FTD2XX):
        return_value = connected_ftdi.get_queue_status()
        assert isinstance(return_value, int)

    def test_get_device_info(self, connected_ftdi: FTD2XX):
        from ftdi_python.ftd2xx import DeviceListInfoNode

        return_value = connected_ftdi.get_device_info()
        assert isinstance(return_value, DeviceListInfoNode)

    def test_get_driver_version(self, connected_ftdi: FTD2XX):
        return_value = connected_ftdi.get_driver_version()
        assert isinstance(return_value, str)

    def test_get_library_version(self, ftdi: FTD2XX):
        return_value = ftdi.get_library_version()
        assert isinstance(return_value, str)

    def test_get_com_port_number(self, connected_ftdi: FTD2XX):
        return_value = connected_ftdi.get_com_port_number()
        assert isinstance(return_value, int)

    def test_get_status(self, connected_ftdi: FTD2XX):
        return_value = connected_ftdi.get_status()
        assert isinstance(return_value, tuple)
        for item in return_value:
            assert isinstance(item, int)

    # set test_set_event_notificiation(self)

    def test_set_chars(self, connected_ftdi: FTD2XX):
        for enable in [False, True, False]:
            connected_ftdi.set_chars(0, enable, 0, enable)

    def test_set_break_on_off(self, connected_ftdi: FTD2XX):
        connected_ftdi.set_break_on()
        connected_ftdi.set_break_on()

    def test_purge(self, connected_ftdi: FTD2XX):
        for enable in [False, True, False]:
            connected_ftdi.purge(enable, enable)

    def test_reset_device(self, connected_ftdi: FTD2XX):
        connected_ftdi.reset_device()

    def test_reset_port(self, connected_ftdi: FTD2XX):
        connected_ftdi.reset_port()

    def test_cycle_port(self, connected_ftdi: FTD2XX):
        import time

        connected_ftdi.cycle_port()
        time.sleep(5)

    def test_rescan(self, connected_ftdi: FTD2XX):
        connected_ftdi.rescan()

    # def test_reload(self)
    # def test_set_reset_pipe_retry_count(self)
    # def test_stop_in_task(self)
    # def test_restart_in_task(self)
    # def test_set_deadman_timeout(self)
    # def test_read_ee(self)
    # def test_write_ee(self)
    # def test_erase_ee(self)
    # def test_ee_read(self)
    # def test_ee_read_ex(self)
    # def test_ee_program(self)
    # def test_ee_program_ex(self)
    # def test_ee_ua_size(self)
    # def test_ee_ua_read(self)
    # def test_ee_ua_write(self)
    # def test_eeprom_read(self)
    # def test_eeprom_program(self)

    def test_get_and_set_latency_timer(self, connected_ftdi: FTD2XX):
        for expected in [0, 100, 255]:
            connected_ftdi.set_latency_timer(expected)
            actual = connected_ftdi.get_latency_timer()
            assert expected == actual

    def test_set_bit_mode(self, connected_ftdi: FTD2XX):
        from ftdi_python.ftd2xx import Bitmodes

        for bit_mode in Bitmodes:
            connected_ftdi.set_bit_mode(0x1F, bit_mode)

    def test_get_bit_mode(self, connected_ftdi: FTD2XX):
        return_type = connected_ftdi.get_bit_mode()
        assert isinstance(return_type, int)

    def test_set_usb_parameters(self, connected_ftdi: FTD2XX):
        connected_ftdi.set_usb_parameters(0, 0)
