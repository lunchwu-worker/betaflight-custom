/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#ifdef USE_PINIO

#include "build/debug.h"

#include "pg/pinio.h"

#include "drivers/io.h"

typedef struct pinioRuntime_s {
    IO_t io;
    bool inverted;
    bool state;
} pinioRuntime_t;

static pinioRuntime_t pinioRuntime[PINIO_COUNT];

#if defined(PINIO1_VBAT_ENABLE_THRESHOLD) && defined(PINIO2_VBAT_ENABLE_THRESHOLD)
#error "Only one VBAT-gated PINIO output is supported"
#elif defined(PINIO1_VBAT_ENABLE_THRESHOLD)
#define PINIO_VBAT_ENABLE_INDEX 0
#define PINIO_VBAT_ENABLE_PIN PINIO1_PIN
#define PINIO_VBAT_ENABLE_THRESHOLD PINIO1_VBAT_ENABLE_THRESHOLD
#elif defined(PINIO2_VBAT_ENABLE_THRESHOLD)
#define PINIO_VBAT_ENABLE_INDEX 1
#define PINIO_VBAT_ENABLE_PIN PINIO2_PIN
#define PINIO_VBAT_ENABLE_THRESHOLD PINIO2_VBAT_ENABLE_THRESHOLD
#endif

#ifdef PINIO_VBAT_ENABLE_THRESHOLD
static bool pinioVbatPresent;
static bool pinioUserDisable;

static void pinioVbatUpdate(void)
{
    const bool enable = pinioVbatPresent && !pinioUserDisable;
    pinioRuntime_t *runtime = &pinioRuntime[PINIO_VBAT_ENABLE_INDEX];
    if (runtime->io && enable != runtime->state) {
        IOWrite(runtime->io, enable);
        runtime->state = enable;
    }
}
#endif

void pinioInit(const pinioConfig_t *pinioConfig)
{
#ifdef PINIO_VBAT_ENABLE_THRESHOLD
    // This target uses one PINIO as an active-high power enable. Ignore
    // persisted PINIO settings and hold it low until VBAT has been sampled.
    IO_t io = IOGetByTag(IO_TAG(PINIO_VBAT_ENABLE_PIN));
    if (io) {
        IOInit(io, OWNER_PINIO, RESOURCE_INDEX(PINIO_VBAT_ENABLE_INDEX));
        IOLo(io);
        IOConfigGPIO(io, IOCFG_OUT_PP);
        pinioRuntime[PINIO_VBAT_ENABLE_INDEX].io = io;
        pinioRuntime[PINIO_VBAT_ENABLE_INDEX].inverted = false;
        pinioRuntime[PINIO_VBAT_ENABLE_INDEX].state = false;
    }
    pinioVbatPresent = false;
    pinioUserDisable = false;
#endif

    for (int i = 0; i < PINIO_COUNT; i++) {
#ifdef PINIO_VBAT_ENABLE_THRESHOLD
        if (i == PINIO_VBAT_ENABLE_INDEX) {
            continue;
        }
#endif
        IO_t io = IOGetByTag(pinioConfig->ioTag[i]);

        if (!io) {
            continue;
        }

        IOInit(io, OWNER_PINIO, RESOURCE_INDEX(i));

        switch (pinioConfig->config[i] & PINIO_CONFIG_MODE_MASK) {
        case PINIO_CONFIG_MODE_OUT_PP:
            // Initial state after reset is input, pull-up.
            // Avoid momentary off by presetting the output to hi.
            if (pinioConfig->config[i] & PINIO_CONFIG_OUT_INVERTED) {
                IOHi(io);
            }
            IOConfigGPIO(io, IOCFG_OUT_PP);
            break;
        }

        if (pinioConfig->config[i] & PINIO_CONFIG_OUT_INVERTED) {
            pinioRuntime[i].inverted = true;
            IOHi(io);
            pinioRuntime[i].state = true;
        } else {
            pinioRuntime[i].inverted = false;
            IOLo(io);
            pinioRuntime[i].state = false;
        }
        pinioRuntime[i].io = io;
    }
}

void pinioSet(int index, bool on)
{
    if (index < 0 || index >= PINIO_COUNT) {
        return;
    }
#ifdef PINIO_VBAT_ENABLE_THRESHOLD
    // USER1 may force the rail off, but cannot bypass the VBAT safety gate.
    if (index == PINIO_VBAT_ENABLE_INDEX) {
        pinioUserDisable = on;
        pinioVbatUpdate();
        return;
    }
#endif
    const bool newState = on ^ pinioRuntime[index].inverted;
    if (newState != pinioRuntime[index].state) {
        IOWrite(pinioRuntime[index].io, newState);
        pinioRuntime[index].state = newState;
    }
}

#ifdef PINIO_VBAT_ENABLE_THRESHOLD
void pinioSetVbat(uint16_t vbat)
{
    pinioVbatPresent = vbat > PINIO_VBAT_ENABLE_THRESHOLD;
    pinioVbatUpdate();
}
#endif
#endif
