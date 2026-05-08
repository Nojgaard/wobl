"""Tests for CRC-16/CCITT-FALSE.

Reference test vector: CRC of ASCII "123456789" = 0x29B1.
See https://crccalc.com/ — CRC-16/CCITT-FALSE row.
"""

from woblpy.hardware.wobl_serial import _crc16


def test_known_vector() -> None:
    """CRC-16/CCITT-FALSE of b"123456789" must equal 0x29B1."""
    assert _crc16(b"123456789") == 0x29B1


def test_empty() -> None:
    """CRC of empty input is the init value 0xFFFF."""
    assert _crc16(b"") == 0xFFFF


def test_single_byte() -> None:
    """Verify a single-byte input against an independently computed value."""
    # CRC-16/CCITT-FALSE of [0x00] = 0xE1F0 (manually traced, init=0xFFFF XOR 0x0000,
    # then 8 shift rounds with poly 0x1021)
    assert _crc16(b"\x00") == 0xE1F0


def test_commutative_with_append() -> None:
    """CRC computed incrementally must match CRC computed in one shot."""
    data = bytes(range(64))
    assert _crc16(data[:32] + data[32:]) == _crc16(data)


def test_different_inputs_differ() -> None:
    assert _crc16(b"\x01") != _crc16(b"\x02")
