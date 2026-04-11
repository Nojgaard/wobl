#pragma once

#include <cstdint>
#include <cstddef>

static size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t ri = 0, wi = 1, ci = 0;
    uint8_t code = 0x01;
    while (ri < len) {
        if (in[ri] == 0x00) {
            out[ci] = code; code = 0x01; ci = wi++;
        } else {
            out[wi++] = in[ri];
            if (++code == 0xFF) { out[ci] = code; code = 0x01; ci = wi++; }
        }
        ri++;
    }
    out[ci] = code;
    return wi;
}

static size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t ri = 0, wi = 0;
    while (ri < len) {
        uint8_t code = in[ri++];
        for (uint8_t i = 1; i < code; i++) out[wi++] = in[ri++];
        if (code < 0xFF && ri < len)       out[wi++] = 0x00;
    }
    return wi;
}