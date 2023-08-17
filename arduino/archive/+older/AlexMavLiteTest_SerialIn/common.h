#pragma once

#include <stdint.h>

// methods for misaligned buffers
static inline uint16_t le16toh_ptr(const uint8_t *p) { return p[0] | (p[1]<<8U); }
static inline uint32_t le32toh_ptr(const uint8_t *p) { return p[0] | (p[1]<<8U) | (p[2]<<16U) | (p[3]<<24U); }

#define PACKED __attribute__((__packed__))

template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
{
    return one < two ? one : two;
}
