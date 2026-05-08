"""Pure-Python COBS (Consistent Overhead Byte Stuffing) codec.

Ported from firmware/include/cobs.hpp — behaviour is bit-for-bit identical.
Encoding removes all 0x00 bytes from the payload; the 0x00 packet delimiter
is written separately by the caller and is NOT included in the return value.
"""


def encode(data: bytes) -> bytes:
    """COBS-encode *data*.  The returned bytes contain no 0x00 bytes."""
    out = bytearray(len(data) + (len(data) // 254) + 2)
    ri = 0
    wi = 1
    ci = 0
    code = 0x01
    while ri < len(data):
        if data[ri] == 0x00:
            out[ci] = code
            code = 0x01
            ci = wi
            wi += 1
        else:
            out[wi] = data[ri]
            wi += 1
            code += 1
            if code == 0xFF:
                out[ci] = code
                code = 0x01
                ci = wi
                wi += 1
        ri += 1
    out[ci] = code
    return bytes(out[:wi])


def decode(data: bytes) -> bytes:
    """COBS-decode *data* (without trailing 0x00 delimiter)."""
    out = bytearray(len(data))
    ri = 0
    wi = 0
    while ri < len(data):
        code = data[ri]
        ri += 1
        for _ in range(1, code):
            out[wi] = data[ri]
            wi += 1
            ri += 1
        if code < 0xFF and ri < len(data):
            out[wi] = 0x00
            wi += 1
    return bytes(out[:wi])
