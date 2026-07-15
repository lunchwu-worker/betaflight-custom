/*
 * PR_F722-BT v1.5 timer mapping for Betaflight 4.4.0
 * DSHOT bitbang mode: motor pins are not mapped here.
 */

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM5,  CH4,  PA3,  TIM_USE_PPM,   0, 0),
    DEF_TIM(TIM3,  CH2,  PA7,  TIM_USE_LED,   0, 0),
    DEF_TIM(TIM13, CH1,  PA6,  TIM_USE_SERVO, 0, 0),
    DEF_TIM(TIM2,  CH3,  PB10, TIM_USE_SERVO, 0, 0),
};
