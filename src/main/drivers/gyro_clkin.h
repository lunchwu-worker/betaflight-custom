#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "drivers/io_types.h"
bool gyroClkInInit(ioTag_t tag, uint32_t freqHz, uint8_t resourceIndex);
