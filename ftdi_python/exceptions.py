# Standard library imports
from __future__ import annotations
from typing import Callable

# Third party imports

# Local application imports


class I2cNackError(Exception):
    """Thrown when a slave does not acknowledge a cmd."""

    def __init__(self, slave_address: int):
        self.slave_address = slave_address
        self.message = f"NACK from slave {slave_address:07b}x."
        super().__init__(self.message)


class LibraryNotFound(Exception):
    """Thrown when the program is unable to load a library."""

    def __init__(
        self,
        library: str,
    ):
        self.library = library
        self.message = f"Unable load {library}."
        super().__init__(self.message)


class ApiCallFailed(Exception):
    """Thrown when the an api call to the FTD2XX.dll fails."""

    def __init__(
        self,
        function: Callable,
        args: tuple,
        status: "FtdiStatus" | int,
    ):
        self.function = function
        self.args = args
        self.status = status
        self.message = (
            f"{function.__name__}{args} failed to execute. Error {str(status)} {status}"
        )
        super().__init__(self.message)


class InvalidHandle(Exception):
    """Thrown when the handle to the ftdi device is invalid."""

    def __init__(
        self,
        message: str = "Invalid ftdi device handle. Make sure you are connected.",
    ):
        self.message = message
        super().__init__(message)


class NoDevicesMatchingSearch(Exception):
    def __init__(
        self,
        search: "DeviceListInfoNode",
    ):
        self.search = search
        self.message = f"No ftdi device match your search."
        super().__init__(self.message, search)


class MultipleDevicesMatchingSearch(Exception):
    def __init__(
        self, search: "DeviceListInfoNode", matching_devices: list["DeviceListInfoNode"]
    ):
        self.search = search
        self.matching_devices = matching_devices
        self.message = f"Multiple ftdi devices match your search."
        super().__init__(self.message, search, matching_devices)
