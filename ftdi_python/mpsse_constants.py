# Standard library imports
from __future__ import annotations

# Third party imports

# Local application imports

# ------------- MPSSE Constants -------------
# MPSSE Control Commands
CMD_SET_DATA_BITS_LOWBYTE = 0x80
CMD_SET_DATA_BITS_HIGHBYTE = 0x82
CMD_GET_DATA_BITS_LOWBYTE = 0x81
CMD_GET_DATA_BITS_HIGHBYTE = 0x83

CMD_SEND_IMMEDIATE = 0x87
CMD_ENABLE_3PHASE_CLOCKING = 0x8C
CMD_DISABLE_3PHASE_CLOCKING = 0x8D
CMD_ENABLE_DRIVE_ONLY_ZERO = 0x9E

# MPSSE Data Command - LSB First
CMD_DATA_LSB_FIRST = 0x08


# MPSSE Data Commands - bit mode - MSB first
CMD_DATA_OUT_BITS_POS_EDGE = 0x12
CMD_DATA_OUT_BITS_NEG_EDGE = 0x13
CMD_DATA_IN_BITS_POS_EDGE = 0x22
CMD_DATA_IN_BITS_NEG_EDGE = 0x26
CMD_DATA_BITS_IN_POS_OUT_NEG_EDGE = 0x33
CMD_DATA_BITS_IN_NEG_OUT_POS_EDGE = 0x36


# MPSSE Data Commands - byte mode - MSB first
CMD_DATA_OUT_BYTES_POS_EDGE = 0x10
CMD_DATA_OUT_BYTES_NEG_EDGE = 0x11
CMD_DATA_IN_BYTES_POS_EDGE = 0x20
CMD_DATA_IN_BYTES_NEG_EDGE = 0x24
CMD_DATA_BYTES_IN_POS_OUT_NEG_EDGE = 0x31
CMD_DATA_BYTES_IN_NEG_OUT_POS_EDGE = 0x34


SEND_ACK = 0x00
SEND_NACK = 0xFF
