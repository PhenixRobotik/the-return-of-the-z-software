#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#include <libopencm3/cm3/common.h>
#define CLEAR(var, mask) ((var) &= ~((uint32_t) mask))
#define SET(var, mask)   ((var) |=  ((uint32_t) mask))
#define IS_SET(var, bit)  ((var) & (bit))

#endif
