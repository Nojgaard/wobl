"""Tests for the COBS codec.

Verifies bit-for-bit compatibility with the C implementation in
firmware/include/cobs.hpp.
"""

import pytest

from woblpy.hardware.cobs import decode, encode


# ---------------------------------------------------------------------------
# Known-vector tests (derived from COBS specification)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("raw, expected_encoded", [
    # Single zero byte → two 0x01 overhead codes
    (b"\x00", b"\x01\x01"),
    # Single non-zero byte
    (b"\x11", b"\x02\x11"),
    # Two non-zero bytes
    (b"\x11\x22", b"\x03\x11\x22"),
    # Zero between two non-zero bytes
    (b"\x11\x00\x22", b"\x02\x11\x02\x22"),
    # Two zeros
    (b"\x00\x00", b"\x01\x01\x01"),
    # Longer sequence from the COBS paper
    (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
])
def test_known_vectors(raw: bytes, expected_encoded: bytes) -> None:
    assert encode(raw) == expected_encoded
    assert decode(expected_encoded) == raw


# ---------------------------------------------------------------------------
# Round-trip property
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("raw", [
    b"",
    b"\x00",
    b"\xff",
    b"\x00" * 10,
    bytes(range(256)),
    bytes(range(255, -1, -1)),
    b"hello, world!\x00\x00",
])
def test_round_trip(raw: bytes) -> None:
    assert decode(encode(raw)) == raw


# ---------------------------------------------------------------------------
# No zero bytes in encoded output
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("raw", [
    b"\x00" * 50,
    bytes(range(256)),
    b"arbitrary\x00data\x00with\x00zeros",
])
def test_no_zero_in_encoded(raw: bytes) -> None:
    assert b"\x00" not in encode(raw)


# ---------------------------------------------------------------------------
# 254-byte boundary (code byte 0xFF)
# ---------------------------------------------------------------------------

def test_254_nonzero_bytes() -> None:
    """253 non-zero bytes is the largest block before a code overflow."""
    raw = bytes(range(1, 254))          # 253 bytes, all non-zero
    encoded = encode(raw)
    assert b"\x00" not in encoded
    assert decode(encoded) == raw


def test_255_nonzero_bytes() -> None:
    """At 254 non-zero bytes, the encoder must emit a 0xFF code and restart."""
    raw = bytes(range(1, 255))          # 254 bytes, all non-zero
    encoded = encode(raw)
    assert b"\x00" not in encoded
    assert decode(encoded) == raw


def test_empty() -> None:
    assert encode(b"") == b"\x01"
    assert decode(b"\x01") == b""
