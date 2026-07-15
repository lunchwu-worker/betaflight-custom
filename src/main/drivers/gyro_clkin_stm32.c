#include "platform.h"
#if defined(USE_GYRO_CLKIN)
#include "drivers/gyro_clkin.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/resource.h"
#include "drivers/timer.h"

static timerChannel_t pwmChannel;
static bool gyroClkInRunning = false;

bool gyroClkInInit(ioTag_t tag, uint32_t freqHz, uint8_t resourceIndex)
{
    if (!tag || freqHz == 0) return false;
    if (gyroClkInRunning) return true;

    const timerHardware_t *timer = timerAllocate(tag, OWNER_GYRO_CLKIN, resourceIndex);
    if (!timer) return false;

    const IO_t io = IOGetByTag(tag);
    if (!io) return false;

    const uint32_t clock = timerClock(timer->tim);
    const uint32_t period = clock / freqHz;
    if (period < 2 || period > UINT16_MAX) return false;

    IOInit(io, OWNER_GYRO_CLKIN, resourceIndex);
    IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);

    const uint16_t period16 = (uint16_t)period;
    const uint16_t duty = period16 / 2;
    pwmOutConfig(&pwmChannel, timer, clock, period16, duty, 0);
    *pwmChannel.ccr = duty;
    gyroClkInRunning = true;
    return true;
}
#endif
