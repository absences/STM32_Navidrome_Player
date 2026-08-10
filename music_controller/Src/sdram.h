#ifndef SDRAM_H
#define SDRAM_H

#include <stdint.h>

#define SDRAM_BASE_ADDR 0xC0000000UL
#define SDRAM_SIZE_BYTES (32UL * 1024UL * 1024UL)

uint32_t SDRAM_InitAndTest(void);

#endif
