# Standard library imports
from __future__ import annotations
import sys
import ctypes

# Third party imports

# Local application imports
import ftdi_python.config as config
from ftdi_python.exceptions import LibraryNotFound

# --- c-types used for the ftd2xx.dll --- #
UINT8 = ctypes.c_uint8
UINT16 = ctypes.c_uint16
UINT32 = ctypes.c_uint32 

INT8 = ctypes.c_int8
INT16 = ctypes.c_int16
INT32 = ctypes.c_int32 

BOOL = ctypes.c_bool
ARRAY = lambda type, size: type * size
STRING = lambda x: ARRAY(ctypes.c_char, x)
PCHAR = ctypes.c_char_p
STRUCT = ctypes.Structure
PVOID = ctypes.c_void_p
FT_HANDLE = PVOID
FT_STATUS = UINT16
FT_DEVICE = UINT32
POINTER = ctypes.POINTER
PASS_BY_REFERENCE = ctypes.byref

__LIBRARY__: ctypes.CDLL


def load_dll(dll_list: list[str]) -> ctypes.CDLL:
    """
    Gets the specified dll.

    Raises:
        LibraryNotFound: On the failure to load the dll.

    Returns:
        ctypes.CDLL: The loaded dll.
    """
    import os

    # os.add_dll_directory(config.DLLS)
    for dll in dll_list:
        library = ctypes.cdll.LoadLibrary(dll)
        if library is not None:
            break

    if library is None:
        raise LibraryNotFound(dll_list)

    return library


if sys.platform == "win32":
    # Windows
    __LIBRARY__ = load_dll(["ftd2xx.dll", "ftd2xx64.dll"])
elif sys.platform == "linux":
    # Linux
    __LIBRARY__ = load_dll(["libftd2xx.so"])


class FT_DEVICE_LIST_INFO_NODE(STRUCT):
    _fields_ = [
        ("Flags", UINT32),
        ("Type", UINT32),
        ("ID", UINT32),
        ("LocId", UINT32),
        ("SerialNumber", STRING(16)),
        ("Description", STRING(64)),
        ("ftHandle", FT_HANDLE),
    ]


# fmt: off
class FT_PROGRAM_DATA_STRUCTURE(STRUCT):
    _fields_ = [
        ("Signature1", UINT32),        # Header - must be 0x0000000
        ("Signature2", UINT32),        # Header - must be 0xffffffff
        ("Version", UINT32),           # Header - FT_PROGRAM_DATA version
                                       # 0 = original (FT232B)
                                       # 1 = FT2232 extensions
                                       # 2 = FT232R extensions
                                       # 3 = FT2232H extensions
                                       # 4 = FT4232H extensions
                                       # 5 = FT232H extensions
        ("VendorId", UINT16),          # 0x0403
        ("ProductId", UINT16),         # 0x6001
        ("Manufacturer", PCHAR),       # "FTDI"
        ("ManufacturerId", PCHAR),     # "FT"
        ("Description", PCHAR),        # "USB HS Serial Converter"
        ("SerialNumber", PCHAR),       # "FT000001" if fixed, or NULL
        ("MaxPower", UINT8),           # 0 < MaxPower <= 500
        ("PnP", UINT8),                # 0 = disabled, 1 = enabled
        ("SelfPowered", UINT8),        # 0 = bus powered, 1 = self powered
        ("RemoteWakeup", UINT8),       # 0 = not capable, 1 = capable
        # Rev4 (FT232B) extensions
        ("Rev4", UINT8),               # non-zero if Rev4 chip, zero otherwise
        ("IsoIn", UINT8),              # non-zero if in endpoint is isochronous
        ("IsoOut", UINT8),             # non-zero if out endpoint is isochronous
        ("PullDownEnable", UINT8),     # non-zero if pull down enabled
        ("SerNumEnable", UINT8),       # non-zero if serial number to be used
        ("USBVersionEnable", UINT8),   # non-zero if chip uses USBVersion
        ("USBVersion", UINT16),        # BCD (0x0200 => USB2)
        # Rev 5 (FT2232) extensions
        ("Rev5", UINT8),               # non-zero if Rev5 chip, zero otherwise
        ("IsoInA", UINT8),             # non-zero if in endpoint is isochronous
        ("IsoInB", UINT8),             # non-zero if in endpoint is isochronous
        ("IsoOutA", UINT8),            # non-zero if out endpoint is isochronous
        ("IsoOutB", UINT8),            # non-zero if out endpoint is isochronous
        ("PullDownEnable5", UINT8),    # non-zero if pull down enabled
        ("SerNumEnable5", UINT8),      # non-zero if serial number to be used
        ("USBVersionEnable5", UINT8),  # non-zero if chip uses USBVersion
        ("USBVersion5", UINT16),       # BCD (0x0200 => USB2)
        ("AIsHighCurrent", UINT8),     # non-zero if interface is high current
        ("BIsHighCurrent", UINT8),     # non-zero if interface is high current
        ("IFAIsFifo", UINT8),          # non-zero if interface is 245 FIFO
        ("IFAIsFifoTar", UINT8),       # non-zero if interface is 245 FIFO CPU target
        ("IFAIsFastSer", UINT8),       # non-zero if interface is Fast serial
        ("AIsVCP", UINT8),             # non-zero if interface is to use VCP drivers
        ("IFBIsFifo", UINT8),          # non-zero if interface is 245 FIFO
        ("IFBIsFifoTar", UINT8),       # non-zero if interface is 245 FIFO CPU target
        ("IFBIsFastSer", UINT8),       # non-zero if interface is Fast serial
        ("BIsVCP", UINT8),             # non-zero if interface is to use VCP drivers
        # Rev 6 (FT232R) extensions
        ("UseExtOsc", UINT8),          # Use External Oscillator
        ("HighDriveIOs", UINT8),       # High Drive I/Os
        ("EndpointSize", UINT8),       # Endpoint size
        ("PullDownEnableR", UINT8),    # non-zero if pull down enabled
        ("SerNumEnableR", UINT8),      # non-zero if serial number to be used
        ("InvertTXD", UINT8),          # non-zero if invert TXD
        ("InvertRXD", UINT8),          # non-zero if invert RXD
        ("InvertRTS", UINT8),          # non-zero if invert RTS
        ("InvertCTS", UINT8),          # non-zero if invert CTS
        ("InvertDTR", UINT8),          # non-zero if invert DTR
        ("InvertDSR", UINT8),          # non-zero if invert DSR
        ("InvertDCD", UINT8),          # non-zero if invert DCD
        ("InvertRI", UINT8),           # non-zero if invert RI
        ("Cbus0", UINT8),              # Cbus Mux control
        ("Cbus1", UINT8),              # Cbus Mux control
        ("Cbus2", UINT8),              # Cbus Mux control
        ("Cbus3", UINT8),              # Cbus Mux control
        ("Cbus4", UINT8),              # Cbus Mux control
        ("RIsD2XX", UINT8),            # non-zero if using D2XX driver
        # Rev 7 (FT2232H) Extensions
        ("PullDownEnable7", UINT8),    # non-zero if pull down enabled
        ("SerNumEnable7", UINT8),      # non-zero if serial number to be used
        ("ALSlowSlew", UINT8),         # non-zero if AL pins have slow slew
        ("ALSchmittInput", UINT8),     # non-zero if AL pins are Schmitt input
        ("ALDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("AHSlowSlew", UINT8),         # non-zero if AH pins have slow slew
        ("AHSchmittInput", UINT8),     # non-zero if AH pins are Schmitt input
        ("AHDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("BLSlowSlew", UINT8),         # non-zero if BL pins have slow slew
        ("BLSchmittInput", UINT8),     # non-zero if BL pins are Schmitt input
        ("BLDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("BHSlowSlew", UINT8),         # non-zero if BH pins have slow slew
        ("BHSchmittInput", UINT8),     # non-zero if BH pins are Schmitt input
        ("BHDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("IFAIsFifo7", UINT8),         # non-zero if interface is 245 FIFO
        ("IFAIsFifoTar7", UINT8),      # non-zero if interface is 245 FIFO CPU target
        ("IFAIsFastSer7", UINT8),      # non-zero if interface is Fast serial
        ("AIsVCP7", UINT8),            # non-zero if interface is to use VCP drivers
        ("IFBIsFifo7", UINT8),         # non-zero if interface is 245 FIFO
        ("IFBIsFifoTar7", UINT8),      # non-zero if interface is 245 FIFO CPU target
        ("IFBIsFastSer7", UINT8),      # non-zero if interface is Fast serial
        ("BIsVCP7", UINT8),            # non-zero if interface is to use VCP drivers
        ("PowerSaveEnable", UINT8),    # non-zero if using BCBUS7 to save power for self-powered
        # Rev 8 (FT4232H) Extensions
        ("PullDownEnable8", UINT8),    # non-zero if pull down enabled
        ("SerNumEnable8", UINT8),      # non-zero if serial number to be used
        ("ASlowSlew", UINT8),          # non-zero if AL pins have slow slew
        ("ASchmittInput", UINT8),      # non-zero if AL pins are Schmitt input
        ("ADriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("BSlowSlew", UINT8),          # non-zero if AH pins have slow slew
        ("BSchmittInput", UINT8),      # non-zero if AH pins are Schmitt input
        ("BDriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("CSlowSlew", UINT8),          # non-zero if BL pins have slow slew
        ("CSchmittInput", UINT8),      # non-zero if BL pins are Schmitt input
        ("CDriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("DSlowSlew", UINT8),          # non-zero if BH pins have slow slew
        ("DSchmittInput", UINT8),      # non-zero if BH pins are Schmitt input
        ("DDriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("ARIIsTXDEN", UINT8),         # non-zero if port A uses RI as RS485 TXDEN
        ("BRIIsTXDEN", UINT8),         # non-zero if port B uses RI as RS485 TXDEN
        ("CRIIsTXDEN", UINT8),         # non-zero if port C uses RI as RS485 TXDEN
        ("DRIIsTXDEN", UINT8),         # non-zero if port D uses RI as RS485 TXDEN
        ("AIsVCP8", UINT8),            # non-zero if interface is to use VCP drivers
        ("BIsVCP8", UINT8),            # non-zero if interface is to use VCP drivers
        ("CIsVCP8", UINT8),            # non-zero if interface is to use VCP drivers
        ("DIsVCP8", UINT8),            # non-zero if interface is to use VCP 
    
        # Rev 9 (FT232H) Extensions
        ("PullDownEnableH", UINT8),    # non-zero if pull down enabled
        ("SerNumEnableH", UINT8),      # non-zero if serial number to be used
        ("ACSlowSlewH", UINT8),        # non-zero if AC pins have slow slew
        ("ACSchmittInputH", UINT8),    # non-zero if AC pins are Schmitt input
        ("ACDriveCurrentH", UINT8),    # valid values are 4mA, 8mA, 12mA, 16mA
        ("ADSlowSlewH", UINT8),        # non-zero if AD pins have slow slew
        ("ADSchmittInputH", UINT8),    # non-zero if AD pins are Schmitt input
        ("ADDriveCurrentH", UINT8),    # valid values are 4mA, 8mA, 12mA, 16mA
        ("Cbus0H", UINT8),             # Cbus Mux control
        ("Cbus1H", UINT8),             # Cbus Mux control
        ("Cbus2H", UINT8),             # Cbus Mux control
        ("Cbus3H", UINT8),             # Cbus Mux control
        ("Cbus4H", UINT8),             # Cbus Mux control
        ("Cbus5H", UINT8),             # Cbus Mux control
        ("Cbus6H", UINT8),             # Cbus Mux control
        ("Cbus7H", UINT8),             # Cbus Mux control
        ("Cbus8H", UINT8),             # Cbus Mux control
        ("Cbus9H", UINT8),             # Cbus Mux control
        ("IsFifoH", UINT8),            # non-zero if interface is 245 FIFO
        ("IsFifoTarH", UINT8),         # non-zero if interface is 245 FIFO CPU target
        ("IsFastSerH", UINT8),         # non-zero if interface is Fast serial
        ("IsFT1248H", UINT8),          # non-zero if interface is FT1248
        ("FT1248CpolH", UINT8),        # FT1248 clock polarity - clock idle high (1) or clock idle low (0)
        ("FT1248LsbH", UINT8),         # FT1248 data is LSB (1) or MSB (0)
        ("FT1248FlowControlH", UINT8), # FT1248 flow control enable
        ("IsVCPH", UINT8),             # non-zero if interface is to use VCP drivers
        ("PowerSaveEnableH", UINT8),   # non-zero if using ACBUS7 to save power for self-powered
    ]
    

class SECURITY_ATTRIBUTES(STRUCT):
    _fields_ = [
        ("nLength", UINT32),
        ("lpSecurityDescriptor", PVOID),
        ("bInheritHandle", BOOL),
    ]

class OVERLAPPED(STRUCT):
    _fields_ = [
        ("Internal", UINT32),
        ("InternalHigh", UINT32),
        ("Offset", UINT32),
        ("OffsetHigh", UINT32),
        ("hEvent", UINT32),
    ]


class FTDCB(STRUCT):
    _fields_ = [
        ("DCBlength", UINT32),
        ("BaudRate", UINT32),
        ("fBinary", UINT32, 1),
        ("fParity", UINT32, 1),
        ("fOutxCtsFlow", UINT32, 1),
        ("fOutxDsrFlow", UINT32, 1),
        ("fDtrControl", UINT32, 2),
        ("fDsrSensitivity", UINT32, 1),
        ("fTXContinueOnXoff", UINT32, 1),
        ("fOutX", UINT32, 1),
        ("fInX", UINT32, 1),
        ("fErrorChar", UINT32, 1),
        ("fNull", UINT32, 1),
        ("fRtsControl", UINT32, 2),
        ("fAbortOnError", UINT32, 1),
        ("fDummy2", UINT32, 17),
        ("wReserved", INT32),
        ("XonLim", INT32),
        ("XoffLim", INT32),
        ("ByteSize", INT8),
        ("Parity", INT8),
        ("StopBits", INT8),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", INT32),
    ]

class FTTIMEOUTS(STRUCT):
    _fields_ = [
        ("ReadIntervalTimeout", UINT32),
        ("ReadTotalTimeoutMultiplier", UINT32),
        ("ReadTotalTimeoutConstant", UINT32),
        ("WriteTotalTimeoutMultiplier", UINT32),
        ("WriteTotalTimeoutConstant", UINT32),
    ]

class FTCOMSTAT(STRUCT):
    _fields_ = [
        ("fCtsHold", UINT32, 1),
        ("fDsrHold", UINT32, 1),
        ("fRlsdHold", UINT32, 1),
        ("fXoffHold", UINT32, 1),
        ("fXoffSent", UINT32, 1),
        ("fEof", UINT32, 1),
        ("fTxim", UINT32, 1),
        ("fReserved", UINT32, 25),
        ("cbInQue", UINT32),
        ("cbOutQue", UINT32),
    ]

# --- D2XX Classic Functions --- #
FT_SetVIDPID = __LIBRARY__.FT_SetVIDPID # Not supported by windows
FT_SetVIDPID.restype = FT_STATUS
FT_SetVIDPID.argtypes = [UINT32, UINT32]

FT_GetVIDPID = __LIBRARY__.FT_GetVIDPID # Not supported by windows
FT_SetVIDPID.restype = FT_STATUS
FT_SetVIDPID.argtypes = [POINTER(UINT32), POINTER(UINT32)]

FT_CreateDeviceInfoList = __LIBRARY__.FT_CreateDeviceInfoList
FT_CreateDeviceInfoList.restype = FT_STATUS
FT_CreateDeviceInfoList.argtypes = [POINTER(UINT32)]

FT_GetDeviceInfoList = __LIBRARY__.FT_GetDeviceInfoList
FT_GetDeviceInfoList.restype = FT_STATUS
FT_GetDeviceInfoList.argtypes = [POINTER(FT_DEVICE_LIST_INFO_NODE), POINTER(UINT32)]

FT_GetDeviceInfoDetail = __LIBRARY__.FT_GetDeviceInfoDetail
FT_GetDeviceInfoDetail.restype = FT_STATUS
FT_GetDeviceInfoDetail.argtypes = [UINT32, POINTER(UINT32), POINTER(UINT32), POINTER(UINT32), POINTER(UINT32), PCHAR, PCHAR, POINTER(FT_HANDLE)]

FT_ListDevices = __LIBRARY__.FT_ListDevices
FT_ListDevices.restype = FT_STATUS
FT_ListDevices.argtypes = [PVOID, PVOID, UINT32]

FT_Open = __LIBRARY__.FT_Open
FT_Open.restype = FT_STATUS
FT_Open.argtypes = [INT32, POINTER(FT_HANDLE)]

FT_OpenEx = __LIBRARY__.FT_OpenEx
FT_OpenEx.restype = FT_STATUS
FT_OpenEx.argtypes = [PVOID, UINT32, POINTER(FT_HANDLE)]

FT_Close = __LIBRARY__.FT_Close
FT_Close.restype = FT_STATUS
FT_Close.argtypes = [FT_HANDLE]

FT_Read = __LIBRARY__.FT_Read
FT_Read.restype = FT_STATUS
FT_Read.argtypes = [FT_HANDLE, POINTER(PVOID), UINT32, POINTER(UINT32)]

FT_Write = __LIBRARY__.FT_Write
FT_Write.restype = FT_STATUS
FT_Write.argtypes = [FT_HANDLE, POINTER(PVOID), UINT32, POINTER(UINT32)]

FT_SetBaudRate = __LIBRARY__.FT_SetBaudRate
FT_SetBaudRate.restype = FT_STATUS
FT_SetBaudRate.argtypes = [FT_HANDLE, UINT32]

FT_SetDivisor = __LIBRARY__.FT_SetDivisor
FT_SetDivisor.restype = FT_STATUS
FT_SetDivisor.argtypes = [FT_HANDLE, UINT16]

FT_SetDataCharacteristics = __LIBRARY__.FT_SetDataCharacteristics
FT_SetDataCharacteristics.restype = FT_STATUS
FT_SetDataCharacteristics.argtypes = [FT_HANDLE, UINT8, UINT8, UINT8]

FT_SetTimeouts = __LIBRARY__.FT_SetTimeouts
FT_SetTimeouts.restype = FT_STATUS
FT_SetTimeouts.argtypes = [FT_HANDLE, UINT32, UINT32]

FT_SetFlowControl = __LIBRARY__.FT_SetFlowControl
FT_SetFlowControl.restype = FT_STATUS
FT_SetFlowControl.argtypes = [FT_HANDLE, UINT16, UINT8, UINT8]

FT_SetDtr = __LIBRARY__.FT_SetDtr
FT_SetDtr.restype = FT_STATUS
FT_SetDtr.argtypes = [FT_HANDLE]

FT_ClrDtr = __LIBRARY__.FT_ClrDtr
FT_ClrDtr.restype = FT_STATUS
FT_ClrDtr.argtypes = [FT_HANDLE]

FT_SetRts = __LIBRARY__.FT_SetRts
FT_SetRts.restype = FT_STATUS
FT_SetRts.argtypes = [FT_HANDLE]

FT_ClrRts = __LIBRARY__.FT_ClrRts
FT_ClrRts.restype = FT_STATUS
FT_ClrRts.argtypes = [FT_HANDLE]

FT_GetModemStatus = __LIBRARY__.FT_GetModemStatus
FT_GetModemStatus.restype = FT_STATUS
FT_GetModemStatus.argtypes = [FT_HANDLE, POINTER(UINT32)]

FT_GetQueueStatus = __LIBRARY__.FT_GetQueueStatus
FT_GetQueueStatus.restype = FT_STATUS
FT_GetQueueStatus.argtypes = [FT_HANDLE, POINTER(UINT32)]

FT_GetDeviceInfo = __LIBRARY__.FT_GetDeviceInfo
FT_GetDeviceInfo.restype = FT_STATUS
FT_GetDeviceInfo.argtypes = [FT_HANDLE, POINTER(FT_DEVICE), POINTER(UINT32), PCHAR, PCHAR, PVOID]

FT_GetDriverVersion = __LIBRARY__.FT_GetDriverVersion # windows only
FT_GetDriverVersion.restype = FT_STATUS
FT_GetDriverVersion.argtypes = [FT_HANDLE, POINTER(UINT32)]

FT_GetLibraryVersion = __LIBRARY__.FT_GetLibraryVersion # windows only
FT_GetLibraryVersion.restype = FT_STATUS
FT_GetLibraryVersion.argtypes = [POINTER(UINT32)]

FT_GetComPortNumber = __LIBRARY__.FT_GetComPortNumber # windows only
FT_GetComPortNumber.restype = FT_STATUS
FT_GetComPortNumber.argtypes = [FT_HANDLE, POINTER(INT32)]

FT_GetStatus = __LIBRARY__.FT_GetStatus
FT_GetStatus.restype = FT_STATUS
FT_GetStatus.argtypes = [FT_HANDLE, POINTER(UINT32), POINTER(UINT32), POINTER(UINT32)]

FT_SetEventNotification = __LIBRARY__.FT_SetEventNotification
FT_SetEventNotification.restype = FT_STATUS
FT_SetEventNotification.argtypes = [FT_HANDLE, UINT32, PVOID]

FT_SetChars = __LIBRARY__.FT_SetChars
FT_SetChars.restype = FT_STATUS
FT_SetChars.argtypes = [FT_HANDLE, UINT8, UINT8, UINT8, UINT8]

FT_SetBreakOn = __LIBRARY__.FT_SetBreakOn
FT_SetBreakOn.restype = FT_STATUS
FT_SetBreakOn.argtypes = [FT_HANDLE]

FT_SetBreakOff = __LIBRARY__.FT_SetBreakOff
FT_SetBreakOff.restype = FT_STATUS
FT_SetBreakOff.argtypes = [FT_HANDLE]

FT_Purge = __LIBRARY__.FT_Purge
FT_Purge.restype = FT_STATUS
FT_Purge.argtypes = [FT_HANDLE, UINT32]

FT_ResetDevice = __LIBRARY__.FT_ResetDevice
FT_ResetDevice.restype = FT_STATUS
FT_ResetDevice.argtypes = [FT_HANDLE]

FT_ResetPort = __LIBRARY__.FT_ResetPort # windows only
FT_ResetPort.restype = FT_STATUS
FT_ResetPort.argtypes = [FT_HANDLE]

FT_CyclePort = __LIBRARY__.FT_CyclePort # windows only
FT_CyclePort.restype = FT_STATUS
FT_CyclePort.argtypes = [FT_HANDLE]

FT_Rescan = __LIBRARY__.FT_Rescan # windows only
FT_Rescan.restype = FT_STATUS
FT_Rescan.argtypes = []

FT_Reload = __LIBRARY__.FT_Reload # windows only
FT_Reload.restype = FT_STATUS
FT_Reload.argtypes = [UINT16, UINT16]

FT_SetResetPipeRetryCount = __LIBRARY__.FT_SetResetPipeRetryCount # windows only
FT_SetResetPipeRetryCount.restype = FT_STATUS
FT_SetResetPipeRetryCount.argtypes = [FT_HANDLE, UINT32]

FT_StopInTask = __LIBRARY__.FT_StopInTask
FT_StopInTask.restype = FT_STATUS
FT_StopInTask.argtypes = [FT_HANDLE]

FT_RestartInTask = __LIBRARY__.FT_RestartInTask
FT_RestartInTask.restype = FT_STATUS
FT_RestartInTask.argtypes = [FT_HANDLE]

FT_SetDeadmanTimeout = __LIBRARY__.FT_SetDeadmanTimeout
FT_SetDeadmanTimeout.restype = FT_STATUS
FT_SetDeadmanTimeout.argtypes = [FT_HANDLE, UINT32]

# FT_IoCtl = __LIBRARY__.FT_IoCtl # undocumented function
# FT_IoCtl.restype = FT_STATUS
# FT_IoCtl.argtypes = []

# FT_SetWaitMask = __LIBRARY__.FT_SetWaitMask # undocumented function
# FT_SetWaitMask.restype = FT_STATUS
# FT_SetWaitMask.argtypes = []

# FT_WaitOnMask = __LIBRARY__.FT_WaitOnMask # undocumented function
# FT_WaitOnMask.restype = FT_STATUS
# FT_WaitOnMask.argtypes = []


# --- EEPROM Programming Interface Functions --- #
FT_ReadEE = __LIBRARY__.FT_ReadEE
FT_ReadEE.restype = FT_STATUS
FT_ReadEE.argtypes = [FT_HANDLE, UINT32, POINTER(UINT32)]

FT_WriteEE = __LIBRARY__.FT_WriteEE
FT_WriteEE.restype = FT_STATUS
FT_WriteEE.argtypes = [FT_HANDLE, UINT32, INT32]

FT_EraseEE = __LIBRARY__.FT_EraseEE
FT_EraseEE.restype = FT_STATUS
FT_EraseEE.argtypes = [FT_HANDLE]

FT_EE_Read = __LIBRARY__.FT_EE_Read
FT_EE_Read.restype = FT_STATUS
FT_EE_Read.argtypes = [FT_HANDLE, POINTER(FT_PROGRAM_DATA_STRUCTURE)]

FT_EE_ReadEx = __LIBRARY__.FT_EE_ReadEx
FT_EE_ReadEx.restype = FT_STATUS
FT_EE_ReadEx.argtypes = [FT_HANDLE, POINTER(FT_PROGRAM_DATA_STRUCTURE), POINTER(INT8), POINTER(INT8), POINTER(INT8), POINTER(INT8)]


FT_EE_Program = __LIBRARY__.FT_EE_Program
FT_EE_Program.restype = FT_STATUS
FT_EE_Program.argtypes = [FT_HANDLE, POINTER(FT_PROGRAM_DATA_STRUCTURE)]


FT_EE_ProgramEx = __LIBRARY__.FT_EE_ProgramEx
FT_EE_ProgramEx.restype = FT_STATUS
FT_EE_ProgramEx.argtypes = [FT_HANDLE, POINTER(FT_PROGRAM_DATA_STRUCTURE), POINTER(INT8), POINTER(INT8), POINTER(INT8), POINTER(INT8)]


FT_EE_UASize = __LIBRARY__.FT_EE_UASize
FT_EE_UASize.restype = FT_STATUS
FT_EE_UASize.argtypes = [FT_HANDLE, POINTER(UINT32)]


FT_EE_UARead = __LIBRARY__.FT_EE_UARead
FT_EE_UARead.restype = FT_STATUS
FT_EE_UARead.argtypes = [FT_HANDLE, POINTER(UINT8), UINT32, POINTER(UINT32)]


FT_EE_UAWrite = __LIBRARY__.FT_EE_UAWrite
FT_EE_UAWrite.restype = FT_STATUS
FT_EE_UAWrite.argtypes = [FT_HANDLE, POINTER(UINT8), UINT32]


FT_EEPROM_Read = __LIBRARY__.FT_EEPROM_Read
FT_EEPROM_Read.restype = FT_STATUS
FT_EEPROM_Read.argtypes = [FT_HANDLE, PVOID, UINT32, POINTER(INT8), POINTER(INT8), POINTER(INT8), POINTER(INT8)]


FT_EEPROM_Program = __LIBRARY__.FT_EEPROM_Program
FT_EEPROM_Program.restype = FT_STATUS
FT_EEPROM_Program.argtypes = [FT_HANDLE, PVOID, UINT32, POINTER(INT8), POINTER(INT8), POINTER(INT8), POINTER(INT8)]


# --- Extended API Functions --- #
FT_SetLatencyTimer = __LIBRARY__.FT_SetLatencyTimer
FT_SetLatencyTimer.restype = FT_STATUS
FT_SetLatencyTimer.argtypes = [FT_HANDLE, UINT8]

FT_GetLatencyTimer = __LIBRARY__.FT_GetLatencyTimer
FT_GetLatencyTimer.restype = FT_STATUS
FT_GetLatencyTimer.argtypes = [FT_HANDLE, POINTER(UINT8)]

FT_SetBitMode = __LIBRARY__.FT_SetBitMode
FT_SetBitMode.restype = FT_STATUS
FT_SetBitMode.argtypes = [FT_HANDLE, UINT8, UINT8]

FT_GetBitMode = __LIBRARY__.FT_GetBitMode
FT_GetBitMode.restype = FT_STATUS
FT_GetBitMode.argtypes = [FT_HANDLE, POINTER(UINT8)]

FT_SetUSBParameters = __LIBRARY__.FT_SetUSBParameters
FT_SetUSBParameters.restype = FT_STATUS
FT_SetUSBParameters.argtypes = [FT_HANDLE, UINT32, UINT32]


# --- FT-Win32 API Functions --- #
FT_W32_CreateFile = __LIBRARY__.FT_W32_CreateFile
FT_W32_CreateFile.restype = FT_STATUS
FT_W32_CreateFile.argtypes = [PVOID, UINT32, UINT32, POINTER(SECURITY_ATTRIBUTES), UINT32, UINT32, PVOID]

FT_W32_CloseHandle = __LIBRARY__.FT_W32_CloseHandle
FT_W32_CloseHandle.restype = FT_STATUS
FT_W32_CloseHandle.argtypes = [FT_HANDLE]

FT_W32_ReadFile = __LIBRARY__.FT_W32_ReadFile
FT_W32_ReadFile.restype = FT_STATUS
FT_W32_ReadFile.argtypes = [FT_HANDLE, PVOID, UINT32, POINTER(UINT32), POINTER(OVERLAPPED)]

FT_W32_WriteFile = __LIBRARY__.FT_W32_WriteFile
FT_W32_WriteFile.restype = FT_STATUS
FT_W32_WriteFile.argtypes = [FT_HANDLE, PVOID, UINT32, POINTER(UINT32), POINTER(OVERLAPPED)]

FT_W32_GetOverlappedResult = __LIBRARY__.FT_W32_GetOverlappedResult
FT_W32_GetOverlappedResult.restype = FT_STATUS
FT_W32_GetOverlappedResult.argtypes = [FT_HANDLE, POINTER(OVERLAPPED), POINTER(UINT32), BOOL]

FT_W32_EscapeCommFunction = __LIBRARY__.FT_W32_EscapeCommFunction
FT_W32_EscapeCommFunction.restype = FT_STATUS
FT_W32_EscapeCommFunction.argtypes = [FT_HANDLE, UINT32]

FT_W32_GetCommModemStatus = __LIBRARY__.FT_W32_GetCommModemStatus
FT_W32_GetCommModemStatus.restype = FT_STATUS
FT_W32_GetCommModemStatus.argtypes = [FT_HANDLE, POINTER(UINT32)]

FT_W32_SetupComm = __LIBRARY__.FT_W32_SetupComm
FT_W32_SetupComm.restype = FT_STATUS
FT_W32_SetupComm.argtypes = [FT_HANDLE, UINT32, UINT32]

FT_W32_SetCommState = __LIBRARY__.FT_W32_SetCommState
FT_W32_SetCommState.restype = FT_STATUS
FT_W32_SetCommState.argtypes = [FT_HANDLE, POINTER(FTDCB)]

FT_W32_GetCommState = __LIBRARY__.FT_W32_GetCommState
FT_W32_GetCommState.restype = FT_STATUS
FT_W32_GetCommState.argtypes = [FT_HANDLE, POINTER(FTDCB)]

FT_W32_SetCommTimeouts = __LIBRARY__.FT_W32_SetCommTimeouts
FT_W32_SetCommTimeouts.restype = FT_STATUS
FT_W32_SetCommTimeouts.argtypes = [FT_HANDLE, POINTER(FTTIMEOUTS)]

FT_W32_GetCommTimeouts = __LIBRARY__.FT_W32_GetCommTimeouts
FT_W32_GetCommTimeouts.restype = FT_STATUS
FT_W32_GetCommTimeouts.argtypes = [FT_HANDLE, POINTER(FTTIMEOUTS)]

FT_W32_SetCommBreak = __LIBRARY__.FT_W32_SetCommBreak
FT_W32_SetCommBreak.restype = FT_STATUS
FT_W32_SetCommBreak.argtypes = [FT_HANDLE]

FT_W32_ClearCommBreak = __LIBRARY__.FT_W32_ClearCommBreak
FT_W32_ClearCommBreak.restype = FT_STATUS
FT_W32_ClearCommBreak.argtypes = [FT_HANDLE]

FT_W32_SetCommMask = __LIBRARY__.FT_W32_SetCommMask
FT_W32_SetCommMask.restype = FT_STATUS
FT_W32_SetCommMask.argtypes = [FT_HANDLE, UINT32]

FT_W32_GetCommMask = __LIBRARY__.FT_W32_GetCommMask
FT_W32_GetCommMask.restype = FT_STATUS
FT_W32_GetCommMask.argtypes = [FT_HANDLE, POINTER(UINT32)]

FT_W32_WaitCommEvent = __LIBRARY__.FT_W32_WaitCommEvent
FT_W32_WaitCommEvent.restype = FT_STATUS
FT_W32_WaitCommEvent.argtypes = [FT_HANDLE, POINTER(UINT32), POINTER(OVERLAPPED)]

FT_W32_PurgeComm = __LIBRARY__.FT_W32_PurgeComm
FT_W32_PurgeComm.restype = FT_STATUS
FT_W32_PurgeComm.argtypes = [FT_HANDLE, UINT32]

FT_W32_GetLastError = __LIBRARY__.FT_W32_GetLastError
FT_W32_GetLastError.restype = FT_STATUS
FT_W32_GetLastError.argtypes = [FT_HANDLE]

FT_W32_ClearCommError = __LIBRARY__.FT_W32_ClearCommError
FT_W32_ClearCommError.restype = FT_STATUS
FT_W32_ClearCommError.argtypes = [FT_HANDLE, POINTER(UINT32), POINTER(FTCOMSTAT)]



class FT_DEVICE_LIST_INFO_NODE(STRUCT):
    _fields_ = [
        ("Flags", UINT32),
        ("Type", UINT32),
        ("ID", UINT32),
        ("LocId", UINT32),
        ("SerialNumber", STRING(16)),
        ("Description", STRING(64)),
        ("ftHandle", FT_HANDLE),
    ]


# fmt: off
class FT_PROGRAM_DATA_STRUCTURE(STRUCT):
    _fields_ = [
        ("Signature1", UINT32),        # Header - must be 0x0000000
        ("Signature2", UINT32),        # Header - must be 0xffffffff
        ("Version", UINT32),           # Header - FT_PROGRAM_DATA version
                                       # 0 = original (FT232B)
                                       # 1 = FT2232 extensions
                                       # 2 = FT232R extensions
                                       # 3 = FT2232H extensions
                                       # 4 = FT4232H extensions
                                       # 5 = FT232H extensions
        ("VendorId", UINT16),          # 0x0403
        ("ProductId", UINT16),         # 0x6001
        ("Manufacturer", PCHAR),       # "FTDI"
        ("ManufacturerId", PCHAR),     # "FT"
        ("Description", PCHAR),        # "USB HS Serial Converter"
        ("SerialNumber", PCHAR),       # "FT000001" if fixed, or NULL
        ("MaxPower", UINT8),           # 0 < MaxPower <= 500
        ("PnP", UINT8),                # 0 = disabled, 1 = enabled
        ("SelfPowered", UINT8),        # 0 = bus powered, 1 = self powered
        ("RemoteWakeup", UINT8),       # 0 = not capable, 1 = capable
        # Rev4 (FT232B) extensions
        ("Rev4", UINT8),               # non-zero if Rev4 chip, zero otherwise
        ("IsoIn", UINT8),              # non-zero if in endpoint is isochronous
        ("IsoOut", UINT8),             # non-zero if out endpoint is isochronous
        ("PullDownEnable", UINT8),     # non-zero if pull down enabled
        ("SerNumEnable", UINT8),       # non-zero if serial number to be used
        ("USBVersionEnable", UINT8),   # non-zero if chip uses USBVersion
        ("USBVersion", UINT16),        # BCD (0x0200 => USB2)
        # Rev 5 (FT2232) extensions
        ("Rev5", UINT8),               # non-zero if Rev5 chip, zero otherwise
        ("IsoInA", UINT8),             # non-zero if in endpoint is isochronous
        ("IsoInB", UINT8),             # non-zero if in endpoint is isochronous
        ("IsoOutA", UINT8),            # non-zero if out endpoint is isochronous
        ("IsoOutB", UINT8),            # non-zero if out endpoint is isochronous
        ("PullDownEnable5", UINT8),    # non-zero if pull down enabled
        ("SerNumEnable5", UINT8),      # non-zero if serial number to be used
        ("USBVersionEnable5", UINT8),  # non-zero if chip uses USBVersion
        ("USBVersion5", UINT16),       # BCD (0x0200 => USB2)
        ("AIsHighCurrent", UINT8),     # non-zero if interface is high current
        ("BIsHighCurrent", UINT8),     # non-zero if interface is high current
        ("IFAIsFifo", UINT8),          # non-zero if interface is 245 FIFO
        ("IFAIsFifoTar", UINT8),       # non-zero if interface is 245 FIFO CPU target
        ("IFAIsFastSer", UINT8),       # non-zero if interface is Fast serial
        ("AIsVCP", UINT8),             # non-zero if interface is to use VCP drivers
        ("IFBIsFifo", UINT8),          # non-zero if interface is 245 FIFO
        ("IFBIsFifoTar", UINT8),       # non-zero if interface is 245 FIFO CPU target
        ("IFBIsFastSer", UINT8),       # non-zero if interface is Fast serial
        ("BIsVCP", UINT8),             # non-zero if interface is to use VCP drivers
        # Rev 6 (FT232R) extensions
        ("UseExtOsc", UINT8),          # Use External Oscillator
        ("HighDriveIOs", UINT8),       # High Drive I/Os
        ("EndpointSize", UINT8),       # Endpoint size
        ("PullDownEnableR", UINT8),    # non-zero if pull down enabled
        ("SerNumEnableR", UINT8),      # non-zero if serial number to be used
        ("InvertTXD", UINT8),          # non-zero if invert TXD
        ("InvertRXD", UINT8),          # non-zero if invert RXD
        ("InvertRTS", UINT8),          # non-zero if invert RTS
        ("InvertCTS", UINT8),          # non-zero if invert CTS
        ("InvertDTR", UINT8),          # non-zero if invert DTR
        ("InvertDSR", UINT8),          # non-zero if invert DSR
        ("InvertDCD", UINT8),          # non-zero if invert DCD
        ("InvertRI", UINT8),           # non-zero if invert RI
        ("Cbus0", UINT8),              # Cbus Mux control
        ("Cbus1", UINT8),              # Cbus Mux control
        ("Cbus2", UINT8),              # Cbus Mux control
        ("Cbus3", UINT8),              # Cbus Mux control
        ("Cbus4", UINT8),              # Cbus Mux control
        ("RIsD2XX", UINT8),            # non-zero if using D2XX driver
        # Rev 7 (FT2232H) Extensions
        ("PullDownEnable7", UINT8),    # non-zero if pull down enabled
        ("SerNumEnable7", UINT8),      # non-zero if serial number to be used
        ("ALSlowSlew", UINT8),         # non-zero if AL pins have slow slew
        ("ALSchmittInput", UINT8),     # non-zero if AL pins are Schmitt input
        ("ALDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("AHSlowSlew", UINT8),         # non-zero if AH pins have slow slew
        ("AHSchmittInput", UINT8),     # non-zero if AH pins are Schmitt input
        ("AHDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("BLSlowSlew", UINT8),         # non-zero if BL pins have slow slew
        ("BLSchmittInput", UINT8),     # non-zero if BL pins are Schmitt input
        ("BLDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("BHSlowSlew", UINT8),         # non-zero if BH pins have slow slew
        ("BHSchmittInput", UINT8),     # non-zero if BH pins are Schmitt input
        ("BHDriveCurrent", UINT8),     # valid values are 4mA, 8mA, 12mA, 16mA
        ("IFAIsFifo7", UINT8),         # non-zero if interface is 245 FIFO
        ("IFAIsFifoTar7", UINT8),      # non-zero if interface is 245 FIFO CPU target
        ("IFAIsFastSer7", UINT8),      # non-zero if interface is Fast serial
        ("AIsVCP7", UINT8),            # non-zero if interface is to use VCP drivers
        ("IFBIsFifo7", UINT8),         # non-zero if interface is 245 FIFO
        ("IFBIsFifoTar7", UINT8),      # non-zero if interface is 245 FIFO CPU target
        ("IFBIsFastSer7", UINT8),      # non-zero if interface is Fast serial
        ("BIsVCP7", UINT8),            # non-zero if interface is to use VCP drivers
        ("PowerSaveEnable", UINT8),    # non-zero if using BCBUS7 to save power for self-powered
        # Rev 8 (FT4232H) Extensions
        ("PullDownEnable8", UINT8),    # non-zero if pull down enabled
        ("SerNumEnable8", UINT8),      # non-zero if serial number to be used
        ("ASlowSlew", UINT8),          # non-zero if AL pins have slow slew
        ("ASchmittInput", UINT8),      # non-zero if AL pins are Schmitt input
        ("ADriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("BSlowSlew", UINT8),          # non-zero if AH pins have slow slew
        ("BSchmittInput", UINT8),      # non-zero if AH pins are Schmitt input
        ("BDriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("CSlowSlew", UINT8),          # non-zero if BL pins have slow slew
        ("CSchmittInput", UINT8),      # non-zero if BL pins are Schmitt input
        ("CDriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("DSlowSlew", UINT8),          # non-zero if BH pins have slow slew
        ("DSchmittInput", UINT8),      # non-zero if BH pins are Schmitt input
        ("DDriveCurrent", UINT8),      # valid values are 4mA, 8mA, 12mA, 16mA
        ("ARIIsTXDEN", UINT8),         # non-zero if port A uses RI as RS485 TXDEN
        ("BRIIsTXDEN", UINT8),         # non-zero if port B uses RI as RS485 TXDEN
        ("CRIIsTXDEN", UINT8),         # non-zero if port C uses RI as RS485 TXDEN
        ("DRIIsTXDEN", UINT8),         # non-zero if port D uses RI as RS485 TXDEN
        ("AIsVCP8", UINT8),            # non-zero if interface is to use VCP drivers
        ("BIsVCP8", UINT8),            # non-zero if interface is to use VCP drivers
        ("CIsVCP8", UINT8),            # non-zero if interface is to use VCP drivers
        ("DIsVCP8", UINT8),            # non-zero if interface is to use VCP 
    
        # Rev 9 (FT232H) Extensions
        ("PullDownEnableH", UINT8),    # non-zero if pull down enabled
        ("SerNumEnableH", UINT8),      # non-zero if serial number to be used
        ("ACSlowSlewH", UINT8),        # non-zero if AC pins have slow slew
        ("ACSchmittInputH", UINT8),    # non-zero if AC pins are Schmitt input
        ("ACDriveCurrentH", UINT8),    # valid values are 4mA, 8mA, 12mA, 16mA
        ("ADSlowSlewH", UINT8),        # non-zero if AD pins have slow slew
        ("ADSchmittInputH", UINT8),    # non-zero if AD pins are Schmitt input
        ("ADDriveCurrentH", UINT8),    # valid values are 4mA, 8mA, 12mA, 16mA
        ("Cbus0H", UINT8),             # Cbus Mux control
        ("Cbus1H", UINT8),             # Cbus Mux control
        ("Cbus2H", UINT8),             # Cbus Mux control
        ("Cbus3H", UINT8),             # Cbus Mux control
        ("Cbus4H", UINT8),             # Cbus Mux control
        ("Cbus5H", UINT8),             # Cbus Mux control
        ("Cbus6H", UINT8),             # Cbus Mux control
        ("Cbus7H", UINT8),             # Cbus Mux control
        ("Cbus8H", UINT8),             # Cbus Mux control
        ("Cbus9H", UINT8),             # Cbus Mux control
        ("IsFifoH", UINT8),            # non-zero if interface is 245 FIFO
        ("IsFifoTarH", UINT8),         # non-zero if interface is 245 FIFO CPU target
        ("IsFastSerH", UINT8),         # non-zero if interface is Fast serial
        ("IsFT1248H", UINT8),          # non-zero if interface is FT1248
        ("FT1248CpolH", UINT8),        # FT1248 clock polarity - clock idle high (1) or clock idle low (0)
        ("FT1248LsbH", UINT8),         # FT1248 data is LSB (1) or MSB (0)
        ("FT1248FlowControlH", UINT8), # FT1248 flow control enable
        ("IsVCPH", UINT8),             # non-zero if interface is to use VCP drivers
        ("PowerSaveEnableH", UINT8),   # non-zero if using ACBUS7 to save power for self-powered
    ]
# fmt: on
