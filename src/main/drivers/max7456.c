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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MAX7456

#include "build/debug.h"

#include "pg/max7456.h"
#include "pg/vcd.h"

#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"
#include "drivers/nvic.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency (boards may override via config.h)
#ifndef MAX7456_MAX_SPI_CLK_HZ
#define MAX7456_MAX_SPI_CLK_HZ 10000000
#endif
#ifndef MAX7456_INIT_MAX_SPI_CLK_HZ
#define MAX7456_INIT_MAX_SPI_CLK_HZ 5000000
#endif
#ifndef MAX7456_POWERUP_DELAY_MS
#define MAX7456_POWERUP_DELAY_MS 50
#endif
#ifndef MAX7456_RESET_READY_TIMEOUT_MS
#define MAX7456_RESET_READY_TIMEOUT_MS 500
#endif

// DEBUG_MAX7456_SIGNAL
#define DEBUG_MAX7456_SIGNAL_MODEREG       0
#define DEBUG_MAX7456_SIGNAL_SENSE         1
#define DEBUG_MAX7456_SIGNAL_REINIT        2
#define DEBUG_MAX7456_SIGNAL_ROWS          3

// DEBUG_MAX7456_SPICLOCK
#define DEBUG_MAX7456_SPICLOCK_OVERCLOCK   0
#define DEBUG_MAX7456_SPICLOCK_DEVTYPE     1
#define DEBUG_MAX7456_SPICLOCK_DIVISOR     2
#define DEBUG_MAX7456_SPICLOCK_X100        3

// VM0 bits
#define VIDEO_BUFFER_DISABLE        0x01
#define MAX7456_RESET               0x02
#define VERTICAL_SYNC_NEXT_VSYNC    0x04
#define OSD_ENABLE                  0x08

#define SYNC_MODE_AUTO              0x00
#define SYNC_MODE_INTERNAL          0x30
#define SYNC_MODE_EXTERNAL          0x20

#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// VM1 bits

// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07

#define BACKGROUND_MODE_GRAY 0x80

// STAT register bits

#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20
#define STAT_RESET_BUSY 0x40

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

// Retained in the ELF for SWD inspection during startup diagnostics.
static volatile uint8_t max7456InitStat;
static volatile uint8_t max7456InitStat2;
static volatile uint8_t max7456InitOsdm;
static volatile uint8_t max7456InitVm0;
static volatile uint8_t max7456InitDmm;
static volatile uint8_t a7456hdGlobalClkSelReadback;
static volatile uint8_t a7456hdHv2Readback;
static volatile uint8_t a7456hdActenReadback;
#ifdef USE_MS7456
static volatile uint8_t max7456InitMsId1;
static volatile uint8_t max7456InitMsId2;
#endif
static volatile uint16_t max7456InitReadyPolls;
static volatile bool max7456InitResetTimedOut;

#ifdef MAX7456_DIAGNOSTIC_LEDS
static IO_t max7456DiagnosticReadyLed = IO_NONE;
static IO_t max7456DiagnosticErrorLed = IO_NONE;

static void max7456DiagnosticLedsInit(void)
{
    if (!max7456DiagnosticReadyLed) {
        max7456DiagnosticReadyLed = IOGetByTag(IO_TAG(MAX7456_DIAGNOSTIC_LED_READY_PIN));
        max7456DiagnosticErrorLed = IOGetByTag(IO_TAG(MAX7456_DIAGNOSTIC_LED_ERROR_PIN));
        IOInit(max7456DiagnosticReadyLed, OWNER_OSD, 0);
        IOInit(max7456DiagnosticErrorLed, OWNER_OSD, 1);
        IOConfigGPIO(max7456DiagnosticReadyLed, IOCFG_OUT_PP);
        IOConfigGPIO(max7456DiagnosticErrorLed, IOCFG_OUT_PP);
    }
}

static void max7456DiagnosticLeds(bool ready, bool warning)
{
    max7456DiagnosticLedsInit();
    // MATEKH743 status LEDs are active low.
    IOWrite(max7456DiagnosticReadyLed, !ready);
    IOWrite(max7456DiagnosticErrorLed, !warning);
}
#else
static void max7456DiagnosticLeds(bool ready, bool warning)
{
    UNUSED(ready);
    UNUSED(warning);
}
#endif

// DMM register bits
#define DMM_AUTO_INC 0x01

// Kluege warning!
// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.

#define VIN_IS_NTSC_alt(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

#define MAX7456_SIGNAL_CHECK_INTERVAL_MS 1000 // msec
#define MAX7456_STALL_CHECK_INTERVAL_MS  1000 // msec

// DMM special bits
#define CLEAR_DISPLAY 0x04
#define CLEAR_DISPLAY_VERT 0x06
#define INVERT_PIXEL_COLOR 0x08

// Special address for terminating incremental write
#define END_STRING 0xff

#define MAX7456ADD_READ         0x80
#define MAX7456ADD_VM0          0x00  //0b0011100// 00 // 00             ,0011100
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#ifdef USE_MS7456
#define MS7456ADD_ID1           0x0e
#define MS7456ADD_ID2           0x0f
#define MS7456_ID1              0x40
#define MS7456_ID2              0x0a
#endif
#define A7456HDADD_ACTEN        0x0f
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_RB1          0x11
#define MAX7456ADD_RB2          0x12
#define MAX7456ADD_RB3          0x13
#define MAX7456ADD_RB4          0x14
#define MAX7456ADD_RB5          0x15
#define MAX7456ADD_RB6          0x16
#define MAX7456ADD_RB7          0x17
#define MAX7456ADD_RB8          0x18
#define MAX7456ADD_RB9          0x19
#define MAX7456ADD_RB10         0x1a
#define MAX7456ADD_RB11         0x1b
#define MAX7456ADD_RB12         0x1c
#define MAX7456ADD_RB13         0x1d
#define MAX7456ADD_RB14         0x1e
#define MAX7456ADD_RB15         0x1f
#define A7456HDADD_HV2          0x55
#define A7456HDADD_SOFTRST      0x60
#define A7456HDADD_GLOBAL_CLK_SEL 0x61
#define MAX7456ADD_OSDBL        0x6c
#define A7456HDADD_STAT2        0x8d
#define MAX7456ADD_STAT         0xA0

#define MS7456_WRITE_PROTECT_TIMEOUT_MS 200

#define NVM_RAM_SIZE            54
#define WRITE_NVR               0xA0

// Device type
#define MAX7456_DEVICE_TYPE_MAX 0
#define MAX7456_DEVICE_TYPE_AT  1
#define MAX7456_DEVICE_TYPE_MS  2

#define CHARS_PER_LINE      30 // XXX Should be related to VIDEO_BUFFER_CHARS_*?

#define MAX7456_SUPPORTED_LAYER_COUNT (DISPLAYPORT_LAYER_BACKGROUND + 1)

typedef struct max7456Layer_s {
    uint8_t buffer[VIDEO_BUFFER_CHARS_PAL];
} max7456Layer_t;

static max7456Layer_t displayLayers[MAX7456_SUPPORTED_LAYER_COUNT];
static displayPortLayer_e activeLayer = DISPLAYPORT_LAYER_FOREGROUND;

extDevice_t max7456Device;
extDevice_t *dev = &max7456Device;

static bool max7456DeviceDetected = false;
static uint16_t max7456SpiClockDiv;
static volatile bool max7456ActiveDma = false;

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

// We write everything to the active layer and then compare
// it with shadowBuffer to update only changed chars.
// This solution is faster then redrawing entire screen.

static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];

//Max bytes to update in one call to max7456DrawScreen()

#define MAX_BYTES2SEND          250
#define MAX_BYTES2SEND_POLLED   12
#define MAX_ENCODE_US           20
#define MAX_ENCODE_US_POLLED    10

static DMA_DATA uint8_t spiBuf[MAX_BYTES2SEND];

static uint8_t  videoSignalCfg;
static uint8_t  videoSignalReg  = OSD_ENABLE; // OSD_ENABLE required to trigger first ReInit
static uint8_t  displayMemoryModeReg = 0;

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static uint8_t  vosRegValue; // VOS (Vertical offset register) value

static bool fontIsLoading       = false;

static uint8_t max7456DeviceType;

static displayPortBackground_e deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;

// previous states initialized outside the valid range to force update on first call
#define INVALID_PREVIOUS_REGISTER_STATE 255
static uint8_t previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
static uint8_t previousInvertRegister = INVALID_PREVIOUS_REGISTER_STATE;

static uint8_t *getLayerBuffer(displayPortLayer_e layer)
{
    return displayLayers[layer].buffer;
}

static uint8_t *getActiveLayerBuffer(void)
{
    return getLayerBuffer(activeLayer);
}

static void max7456SetRegisterVM1(void)
{
    uint8_t backgroundGray = BACKGROUND_BRIGHTNESS_28; // this is the device default background gray level
    uint8_t vm1Register = BLINK_TIME_1 | BLINK_DUTY_CYCLE_75_25; // device defaults
    if (deviceBackgroundType != DISPLAY_BACKGROUND_TRANSPARENT) {
        vm1Register |= BACKGROUND_MODE_GRAY;
        switch (deviceBackgroundType) {
        case DISPLAY_BACKGROUND_BLACK:
            backgroundGray = BACKGROUND_BRIGHTNESS_0;
            break;
        case DISPLAY_BACKGROUND_LTGRAY:
            backgroundGray = BACKGROUND_BRIGHTNESS_49;
            break;
        case DISPLAY_BACKGROUND_GRAY:
        default:
            backgroundGray = BACKGROUND_BRIGHTNESS_28;
            break;
        }
    }
    vm1Register |= (backgroundGray << 4);
    spiWriteReg(dev, MAX7456ADD_VM1, vm1Register);
}

uint8_t max7456GetRowsCount(void)
{
    return (videoSignalReg & VIDEO_MODE_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
}

// When clearing the shadow buffer we fill with 0 so that the characters will
// be flagged as changed when compared to the 0x20 used in the layer buffers.
static void max7456ClearShadowBuffer(void)
{
    memset(shadowBuffer, 0, maxScreenSize);
}

// Buffer is filled with the whitespace character (0x20)
static void max7456ClearLayer(displayPortLayer_e layer)
{
    memset(getLayerBuffer(layer), 0x20, VIDEO_BUFFER_CHARS_PAL);
}

static void max7456ReInit(void)
{
    uint8_t srdata = 0;

    switch (videoSignalCfg) {
    case VIDEO_SYSTEM_PAL:
        videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        break;

    case VIDEO_SYSTEM_NTSC:
        videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        break;

    case VIDEO_SYSTEM_AUTO:
        srdata = spiReadRegMsk(dev, MAX7456ADD_STAT);

        if (VIN_IS_NTSC(srdata)) {
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        } else if (VIN_IS_PAL(srdata)) {
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        } else {
            // No valid input signal, fallback to default (PAL)
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        }
        break;
    }

#ifdef MAX7456_FORCE_INTERNAL_SYNC
    videoSignalReg |= SYNC_MODE_INTERNAL;
#endif

    if (videoSignalReg & VIDEO_MODE_PAL) { //PAL
        maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
    }

    // Set all rows to same charactor black/white level
    previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
    max7456Brightness(0, 2);
    // Re-enable MAX7456 (last function call disables it)

    // Make sure the Max7456 is enabled
    spiWriteReg(dev, MAX7456ADD_VM0, videoSignalReg);
    spiWriteReg(dev, MAX7456ADD_HOS, hosRegValue);
    spiWriteReg(dev, MAX7456ADD_VOS, vosRegValue);

    max7456SetRegisterVM1();

    // Clear shadow to force redraw all screen
    max7456ClearShadowBuffer();
}

void max7456Preinit(const max7456Config_t *max7456Config)
{
    ioPreinitByTag(max7456Config->csTag, max7456Config->preInitOPU ? IOCFG_OUT_PP : IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
}

#ifdef USE_MS7456
static bool ms7456ClockConfig(bool inputMode, uint16_t htotal, uint16_t pixelClock)
{
    static const uint8_t dividers[] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 24, 28, 36, 40, 48, 56, 72, 80, 112, 144
    };
    static const uint8_t dividerRegisters[] = {
        0x00, 0x01, 0x24, 0x81, 0x40, 0xA4, 0x48, 0x82, 0x74, 0x41, 0xA5, 0xC8, 0x83,
        0xF4, 0xC1, 0xA6, 0xC9, 0xF5, 0xC2, 0xA7, 0xCA, 0xF6, 0xC3, 0xCB, 0xF7
    };
    uint8_t inputSelect;
    uint8_t inputConfig;
    int dividerIndex;
    uint32_t fvco = 0;

    spiWriteReg(dev, 0x6E, 0x00);
    spiWriteReg(dev, 0x6D, 0x10);
    spiWriteReg(dev, 0x6E, 0x80);

    if (inputMode) {
        inputSelect = 0;
        inputConfig = 0x01;
    } else {
        inputSelect = 0x02;
        inputConfig = 0x09;
        htotal = (pixelClock * 10000U / 16875U) >> 2;
        spiWriteReg(dev, 0x7B, 0x64);
    }

    htotal--;
    spiWriteReg(dev, 0x77, htotal);
    spiWriteReg(dev, 0x78, htotal >> 8);

    for (dividerIndex = ARRAYLEN(dividers) - 1; dividerIndex >= 0; dividerIndex--) {
        fvco = (uint32_t)pixelClock * dividers[dividerIndex];
        if (fvco >= 60000U && fvco < 84000U) {
            break;
        }
    }
    if (dividerIndex < 0) {
        return false;
    }

    spiWriteReg(dev, 0x6D, inputSelect);
    spiWriteReg(dev, 0x7C, inputConfig);
    spiWriteReg(dev, 0x70, dividerRegisters[dividerIndex]);

    const uint8_t feedbackDivider = (27000000U / 10000U) / 100U / 3U - 1U;
    spiWriteReg(dev, 0x71, feedbackDivider);

    const uint16_t loopDivider = fvco * (feedbackDivider + 1U) / (27000000U / 10000U) - 5U;
    uint8_t htotalHigh = spiReadRegMsk(dev, 0x78) & 0x0F;
    spiWriteReg(dev, 0x78, (loopDivider << 4) | htotalHigh);
    spiWriteReg(dev, 0x79, loopDivider >> 4);

    uint8_t postDivider;
    if (dividers[dividerIndex] <= 7) {
        postDivider = 1;
    } else if (dividers[dividerIndex] <= 14) {
        postDivider = 2;
    } else {
        postDivider = 3;
    }
    spiWriteReg(dev, 0x74, postDivider);
    spiWriteReg(dev, 0x76, 0x2D);
    spiWriteReg(dev, 0x75, 0x0C);
    spiWriteReg(dev, 0x6D, 0x40 | inputSelect);
    spiWriteReg(dev, 0x6E, 0x81);
    delay(10);

    return true;
}

static void ms7456InputSignalDetectService(void)
{
    // Force configuration on the first valid sample. A zero initializer would
    // skip the first NTSC/no-signal-low state because STAT[2:0] can also be 0.
    static uint8_t lastInputMode = UINT8_MAX;
    static uint8_t serviceDivider;

    if (!max7456DeviceDetected || ++serviceDivider < 100) {
        return;
    }
    serviceDivider = 0;

    spiWait(dev);
    uint8_t inputMode = spiReadRegMsk(dev, MAX7456ADD_STAT) & 0x07;
    if (inputMode == lastInputMode) {
        return;
    }

    delay(10);
    inputMode = spiReadRegMsk(dev, MAX7456ADD_STAT) & 0x07;
    if (inputMode == lastInputMode) {
        return;
    }
    lastInputMode = inputMode;

    if (inputMode & STAT_LOS) {
        return;
    }

    uint8_t vm0 = spiReadRegMsk(dev, MAX7456ADD_VM0) & ~OSD_ENABLE;
    spiWriteReg(dev, MAX7456ADD_VM0, vm0);
    spiWriteReg(dev, 0x7D, 0x00);

    if (inputMode & STAT_PAL) {
        vm0 |= VIDEO_MODE_PAL;
        ms7456ClockConfig(true, 1728, 2700);
    } else {
        vm0 &= ~VIDEO_MODE_PAL;
        ms7456ClockConfig(true, 1716, 2700);
    }

    spiWriteReg(dev, 0x7D, 0x01);
    spiWriteReg(dev, MAX7456ADD_VM0, vm0 | OSD_ENABLE);
}

static bool ms7456WaitForWriteProtect(void)
{
    for (unsigned elapsedMs = 0; elapsedMs < MS7456_WRITE_PROTECT_TIMEOUT_MS; elapsedMs++) {
        delay(1);
        if (spiReadRegMsk(dev, 0x62) & 0x10) {
            return true;
        }
    }
    return false;
}
#endif

// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)

max7456InitStatus_e max7456Init(const max7456Config_t *max7456Config, const vcdProfile_t *pVcdProfile, bool cpuOverclock)
{
    max7456DeviceDetected = false;
    deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;
    // Error LED stays on throughout initialization. If both LEDs remain off,
    // this function was never entered; if only error remains on, init stalled
    // or failed before completion.
    max7456DiagnosticLeds(false, true);

    // initialize all layers
    for (unsigned i = 0; i < MAX7456_SUPPORTED_LAYER_COUNT; i++) {
        max7456ClearLayer(i);
    }

    max7456HardwareReset();

    if (!max7456Config->csTag || !spiSetBusInstance(dev, max7456Config->spiDevice)) {
        max7456DiagnosticLeds(false, true);
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    dev->busType_u.spi.csnPin = IOGetByTag(max7456Config->csTag);

    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)) {
        max7456DiagnosticLeds(false, true);
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);

    // Detect MAX7456 existence and device type. Do this at half the speed for safety.

    // Detect MAX7456 and compatible devices. MAX/AT parts are identified by
    // the default OSDM value. MS7456 is identified by its dedicated ID
    // registers because its initialization changes OSDM to 0x00 and that
    // value survives an MCU-only reset.

    spiSetClkDivisor(dev, spiCalculateDivider(MAX7456_INIT_MAX_SPI_CLK_HZ));

    // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
    spiWrite(dev, END_STRING);

#if !defined(MAX7456_OFFICIAL_INIT_LOGIC) && !defined(USE_MS7456)
    // Record power-on status for SWD diagnostics, but do not gate startup on
    // STAT[6]. Some compatible devices report this bit incorrectly.
    max7456InitStat = spiReadRegMsk(dev, MAX7456ADD_STAT);
    max7456InitReadyPolls = 1;
    max7456InitResetTimedOut = (max7456InitStat & STAT_RESET_BUSY) != 0;
#endif

    max7456InitOsdm = spiReadRegMsk(dev, MAX7456ADD_OSDM);

    // The reference implementation first verifies the AT-style CA[8] bit,
    // then effectively identifies MS7456 by ID1 alone. Its ID2 expression is
    // an assignment, so the advertised 0x0A value was never a real gate.
    spiWriteReg(dev, MAX7456ADD_CMAL, (1 << 6));
    const bool supportsCa8 = (spiReadRegMsk(dev, MAX7456ADD_CMAL) & (1 << 6)) != 0;
#ifdef USE_MS7456
    max7456InitMsId1 = spiReadRegMsk(dev, MS7456ADD_ID1);
    max7456InitMsId2 = spiReadRegMsk(dev, MS7456ADD_ID2);
    const bool isMs7456 = supportsCa8 && max7456InitMsId1 == MS7456_ID1;
#else
    const bool isMs7456 = false;
#endif

    if (max7456InitOsdm != 0x1B && !isMs7456) {
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        max7456DiagnosticLeds(false, true);
        return MAX7456_INIT_NOT_FOUND;
    }

    // At this point, we can claim the ownership of the CS pin
    max7456DeviceDetected = true;
    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);

    if (isMs7456) {
        max7456DeviceType = MAX7456_DEVICE_TYPE_MS;
    } else if (supportsCa8) {
        max7456DeviceType = MAX7456_DEVICE_TYPE_AT;
    } else {
        max7456DeviceType = MAX7456_DEVICE_TYPE_MAX;
    }

#if defined(MAX7456_OFFICIAL_INIT_LOGIC) && !defined(USE_OVERCLOCK)
    UNUSED(max7456Config);
    UNUSED(cpuOverclock);
    max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
#else
    // Determine SPI clock divisor based on config.
    // HALF = 5 MHz, NOMINAL = 10 MHz, DOUBLE = 20 MHz.
    switch (max7456Config->clockConfig) {
    case MAX7456_CLOCK_CONFIG_HALF:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ / 2);
        break;

    case MAX7456_CLOCK_CONFIG_NOMINAL:
    default:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
        break;

    case MAX7456_CLOCK_CONFIG_DOUBLE:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ * 2);
        break;
    }

    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_OVERCLOCK, cpuOverclock);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DEVTYPE, max7456DeviceType);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DIVISOR, max7456SpiClockDiv);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_X100, spiCalculateClock(max7456SpiClockDiv) / 10000);
#endif

    spiSetClkDivisor(dev, max7456SpiClockDiv);

#ifdef A7456HD_GLOBAL_SOFT_RESET
    spiWriteReg(dev, A7456HDADD_ACTEN, 0xff);
    spiWriteReg(dev, A7456HDADD_SOFTRST, 0x55);
    delay(50);
    spiWriteReg(dev, A7456HDADD_ACTEN, 0x00);
#endif

    // force soft reset on Max7456
    spiWriteReg(dev, MAX7456ADD_VM0, MAX7456_RESET);

    // Wait for 200us before polling for completion of reset.
    delayMicroseconds(200);

    bool resetComplete = false;
    for (unsigned elapsedMs = 0; elapsedMs < MAX7456_RESET_READY_TIMEOUT_MS; elapsedMs++) {
        if ((spiReadRegMsk(dev, MAX7456ADD_VM0) & MAX7456_RESET) == 0x00) {
            resetComplete = true;
            break;
        }
        delay(1);
    }
    if (!resetComplete) {
        max7456DeviceDetected = false;
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        max7456DiagnosticLeds(false, true);
        return MAX7456_INIT_NOT_FOUND;
    }

    // Capture the other status registers only after reset, without making
    // them startup gates.
#ifndef MAX7456_OFFICIAL_INIT_LOGIC
    max7456InitVm0 = spiReadRegMsk(dev, MAX7456ADD_VM0);
    max7456InitDmm = spiReadRegMsk(dev, MAX7456ADD_DMM);
    max7456InitStat = spiReadRegMsk(dev, MAX7456ADD_STAT);
    max7456InitStat2 = spiReadRegMsk(dev, A7456HDADD_STAT2);
#endif

#ifdef A7456HD_27MHZ_CLOCK_INIT
    spiWriteReg(dev, A7456HDADD_ACTEN, 0xff);
    spiWriteReg(dev, A7456HDADD_GLOBAL_CLK_SEL, 0x00);
    spiWriteReg(dev, A7456HDADD_HV2, 0x86);
    spiWriteReg(dev, A7456HDADD_ACTEN, 0x00);

    // Re-open the extended bank only for readback verification.
    spiWriteReg(dev, A7456HDADD_ACTEN, 0xff);
    a7456hdGlobalClkSelReadback = spiReadRegMsk(dev, A7456HDADD_GLOBAL_CLK_SEL);
    a7456hdHv2Readback = spiReadRegMsk(dev, A7456HDADD_HV2);
    spiWriteReg(dev, A7456HDADD_ACTEN, 0x00);
    a7456hdActenReadback = spiReadRegMsk(dev, A7456HDADD_ACTEN);
#endif

    // Setup values to write to registers
    videoSignalCfg = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;

#ifdef USE_MS7456
    if (max7456DeviceType == MAX7456_DEVICE_TYPE_MS) {
        spiWriteReg(dev, 0x5D, 0x02);
        spiWriteReg(dev, MAX7456ADD_OSDM, 0x00);
        spiWriteReg(dev, 0x58, 0x45);
        spiWriteReg(dev, 0x61, 0x13);
    }
#endif

    // Both LEDs on means the compatible part passed all functional checks,
    // but its STAT[6] power-on-reset indication never cleared.
    max7456DiagnosticLeds(true, max7456InitResetTimedOut);

    // Real init will be made later when driver detect idle.
    return MAX7456_INIT_OK;
}

/**
 * Sets inversion of black and white pixels.
 */
void max7456Invert(bool invert)
{
    if (invert) {
        displayMemoryModeReg |= INVERT_PIXEL_COLOR;
    } else {
        displayMemoryModeReg &= ~INVERT_PIXEL_COLOR;
    }

    if (displayMemoryModeReg != previousInvertRegister) {
        // clear the shadow buffer so all characters will be
        // redrawn with the proper invert state
        max7456ClearShadowBuffer();
        previousInvertRegister = displayMemoryModeReg;
        spiWriteReg(dev, MAX7456ADD_DMM, displayMemoryModeReg);
    }
}

/**
 * Sets the brightness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
void max7456Brightness(uint8_t black, uint8_t white)
{
    const uint8_t reg = (black << 2) | (3 - white);

    if (reg != previousBlackWhiteRegister) {
        previousBlackWhiteRegister = reg;
        STATIC_DMA_DATA_AUTO uint8_t buf[32];
        for (int i = MAX7456ADD_RB0, j = 0; i <= MAX7456ADD_RB15; i++) {
            buf[j++] = i;
            buf[j++] = reg;
        }
        spiReadWriteBuf(dev, buf, NULL, sizeof(buf));
    }
}

void max7456ClearScreen(void)
{
    max7456ClearLayer(activeLayer);
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t *buffer = getActiveLayerBuffer();
    if (x < CHARS_PER_LINE && y < VIDEO_LINES_PAL) {
        buffer[y * CHARS_PER_LINE + x] = c;
    }
}

void max7456Write(uint8_t x, uint8_t y, const char *text)
{
    if (y < VIDEO_LINES_PAL) {
        uint8_t *buffer = getActiveLayerBuffer();
        const uint32_t bufferYOffset = y * CHARS_PER_LINE;
        for (int i = 0, bufferXOffset = x; text[i] && bufferXOffset < CHARS_PER_LINE; i++, bufferXOffset++) {
            buffer[bufferYOffset + bufferXOffset] = text[i];
        }
    }
}

bool max7456LayerSupported(displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND) {
        return true;
    } else {
        return false;
    }
}

bool max7456LayerSelect(displayPortLayer_e layer)
{
    if (max7456LayerSupported(layer)) {
        activeLayer = layer;
        return true;
    } else {
        return false;
    }
}

bool max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    if ((sourceLayer != destLayer) && max7456LayerSupported(sourceLayer) && max7456LayerSupported(destLayer)) {
        memcpy(getLayerBuffer(destLayer), getLayerBuffer(sourceLayer), VIDEO_BUFFER_CHARS_PAL);
        return true;
    } else {
        return false;
    }
}

bool max7456DmaInProgress(void)
{
    return max7456ActiveDma;
}

bool max7456BuffersSynced(void)
{
    for (int i = 0; i < maxScreenSize; i++) {
        if (displayLayers[DISPLAYPORT_LAYER_FOREGROUND].buffer[i] != shadowBuffer[i]) {
            return false;
        }
    }
    return true;
}

bool max7456ReInitIfRequired(bool forceStallCheck)
{
    static timeMs_t lastSigCheckMs = 0;
    static timeMs_t videoDetectTimeMs = 0;
    static uint16_t reInitCount = 0;
    static timeMs_t lastStallCheckMs = MAX7456_STALL_CHECK_INTERVAL_MS / 2; // offset so that it doesn't coincide with the signal check

    const timeMs_t nowMs = millis();

    bool stalled = false;
    if (forceStallCheck || (lastStallCheckMs + MAX7456_STALL_CHECK_INTERVAL_MS < nowMs)) {
        lastStallCheckMs = nowMs;

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        spiWrite(dev, END_STRING);

        stalled = (spiReadRegMsk(dev, MAX7456ADD_VM0) != videoSignalReg);
    }

    if (stalled) {
        max7456ReInit();
    } else if ((videoSignalCfg == VIDEO_SYSTEM_AUTO)
              && ((nowMs - lastSigCheckMs) > MAX7456_SIGNAL_CHECK_INTERVAL_MS)) {

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        spiWrite(dev, END_STRING);

        // Adjust output format based on the current input format.

        const uint8_t videoSense = spiReadRegMsk(dev, MAX7456ADD_STAT);

        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_MODEREG, videoSignalReg & VIDEO_MODE_MASK);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_SENSE, videoSense & 0x7);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_ROWS, max7456GetRowsCount());

        if (videoSense & STAT_LOS) {
            videoDetectTimeMs = 0;
        } else {
            if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(videoSignalReg))
              || (VIN_IS_NTSC_alt(videoSense) && VIDEO_MODE_IS_PAL(videoSignalReg))) {
                if (videoDetectTimeMs) {
                    if (millis() - videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
                        max7456ReInit();
                        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_REINIT, ++reInitCount);
                    }
                } else {
                    // Wait for signal to stabilize
                    videoDetectTimeMs = millis();
                }
            }
        }

        lastSigCheckMs = nowMs;
    }

    max7456DiagnosticLeds(max7456DeviceDetected && !stalled, max7456InitResetTimedOut || stalled);

    return stalled;
}

// Called in ISR context
static busStatus_e max7456_callbackReady(uintptr_t arg)
{
    UNUSED(arg);

    max7456ActiveDma = false;

    return BUS_READY;
}

// Return true if screen still being transferred
bool max7456DrawScreen(void)
{
    static uint16_t pos = 0;
    // This routine doesn't block so need to use static data
    static busSegment_t segments[] = {
            {.u.link = {NULL, NULL}, 0, true, max7456_callbackReady},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

#ifdef USE_MS7456
    if (max7456DeviceType == MAX7456_DEVICE_TYPE_MS) {
        if (fontIsLoading) {
            return false;
        }
        if (spiIsBusy(dev)) {
            return true;
        }

        ms7456InputSignalDetectService();
        uint8_t *buffer = getActiveLayerBuffer();

        for (uint16_t index = 0; index < maxScreenSize; index++) {
            if (buffer[index] == shadowBuffer[index]) {
                continue;
            }
            if (buffer[index] == 0xff) {
                buffer[index] = ' ';
            }

            // MS7456 requires separate SPI transactions and OSD gating for
            // every display-memory write.
            const uint8_t vm0 = spiReadRegMsk(dev, MAX7456ADD_VM0);
            spiWriteReg(dev, MAX7456ADD_VM0, vm0 & ~OSD_ENABLE);
            spiWriteReg(dev, MAX7456ADD_DMAH, index >> 8);
            spiWriteReg(dev, MAX7456ADD_DMAL, index & 0xff);
            spiWriteReg(dev, MAX7456ADD_DMDI, buffer[index]);
            shadowBuffer[index] = buffer[index];
            spiWriteReg(dev, MAX7456ADD_VM0, vm0 | OSD_ENABLE);
        }

        return false;
    }
#endif

#ifdef MAX7456_FORCE_SINGLE_REGISTER_WRITES
    if (!fontIsLoading) {
        uint8_t *buffer = getActiveLayerBuffer();

        for (uint16_t index = 0; index < maxScreenSize; index++) {
            if (buffer[index] == shadowBuffer[index]) {
                continue;
            }
            if (buffer[index] == 0xff) {
                buffer[index] = ' ';
            }

            spiWriteReg(dev, MAX7456ADD_DMM, displayMemoryModeReg);
            spiWriteReg(dev, MAX7456ADD_DMAH, index >> 8);
            spiWriteReg(dev, MAX7456ADD_DMAL, index & 0xff);
            spiWriteReg(dev, MAX7456ADD_DMDI, buffer[index]);
            shadowBuffer[index] = buffer[index];
        }
    }

    return false;
#endif

    if (!fontIsLoading) {
        uint8_t *buffer = getActiveLayerBuffer();
        int spiBufIndex = 0;
        int maxSpiBufStartIndex;
        timeDelta_t maxEncodeTime;
        bool setAddress = true;
        bool autoInc = false;
        int posLimit = pos + (maxScreenSize / 2);

#ifdef USE_DMA
        const bool useDma = spiUseSDO_DMA(dev);
#else
        const bool useDma = false;
#endif
        maxSpiBufStartIndex = useDma ? MAX_BYTES2SEND : MAX_BYTES2SEND_POLLED;
        maxEncodeTime = useDma ? MAX_ENCODE_US : MAX_ENCODE_US_POLLED;

        // Abort for now if the bus is still busy
        if (spiIsBusy(dev)) {
            // Not finished yet
            return true;
        }

        timeUs_t startTime = micros();

        // Allow for an ESCAPE, a reset of DMM and a two byte MAX7456ADD_DMM command at end of buffer
        maxSpiBufStartIndex -= 4;

        // Initialise the transfer buffer
        while ((spiBufIndex < maxSpiBufStartIndex) && (pos < posLimit) && (cmpTimeUs(micros(), startTime) < maxEncodeTime)) {
            if (buffer[pos] != shadowBuffer[pos]) {
                if (buffer[pos] == 0xff) {
                    buffer[pos] = ' ';
                }

                if (setAddress || !autoInc) {
                    if (buffer[pos + 1] != shadowBuffer[pos + 1]) {
                        // It's worth auto incrementing
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        spiBuf[spiBufIndex++] = displayMemoryModeReg | DMM_AUTO_INC;
                        autoInc = true;
                    } else {
                        // It's not worth auto incrementing
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        spiBuf[spiBufIndex++] = displayMemoryModeReg;
                        autoInc = false;
                    }

                    spiBuf[spiBufIndex++] = MAX7456ADD_DMAH;
                    spiBuf[spiBufIndex++] = pos >> 8;
                    spiBuf[spiBufIndex++] = MAX7456ADD_DMAL;
                    spiBuf[spiBufIndex++] = pos & 0xff;

                    setAddress = false;
                }

                spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                spiBuf[spiBufIndex++] = buffer[pos];

                shadowBuffer[pos] = buffer[pos];
            } else {
                if (!setAddress) {
                    setAddress = true;
                    if (autoInc) {
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                        spiBuf[spiBufIndex++] = END_STRING;
                    }
                }
            }

            if (++pos >= maxScreenSize) {
                pos = 0;
                break;
            }
        }

        if (autoInc) {
            if (!setAddress) {
                spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                spiBuf[spiBufIndex++] = END_STRING;
            }

            spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
            spiBuf[spiBufIndex++] = displayMemoryModeReg;
        }

        if (spiBufIndex) {
            segments[0].u.buffers.txData = spiBuf;
            segments[0].len = spiBufIndex;

            max7456ActiveDma = true;

            spiSequence(dev, &segments[0]);

            // Non-blocking, so transfer still in progress if using DMA
        }
    }

    return (pos != 0);
}

// should not be used when armed
void max7456RefreshAll(void)
{
    max7456ReInitIfRequired(true);
    while (max7456DrawScreen());
}

bool max7456WriteNvm(uint8_t char_address, const uint8_t *font_data)
{
    if (!max7456DeviceDetected) {
        return false;
    }

    // Block pending completion of any prior SPI access
    spiWait(dev);

    // disable display
    fontIsLoading = true;
    spiWriteReg(dev, MAX7456ADD_VM0, 0);

#ifdef USE_MS7456
    if (max7456DeviceType == MAX7456_DEVICE_TYPE_MS) {
        spiWriteReg(dev, 0x5D, 0x03);
        spiWriteReg(dev, 0x64, 0x00);
        spiWriteReg(dev, 0x62, 0x02);
        if (!ms7456WaitForWriteProtect()) {
            spiWriteReg(dev, 0x5D, 0x02);
            return false;
        }
    }
#endif

    spiWriteReg(dev, MAX7456ADD_CMAH, char_address); // set start address high

    for (int x = 0; x < 54; x++) {
        spiWriteReg(dev, MAX7456ADD_CMAL, x); //set start address low
        spiWriteReg(dev, MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // Transfer 54 bytes from shadow ram to NVM

    spiWriteReg(dev, MAX7456ADD_CMM, WRITE_NVR);

    // Wait until bit 5 in the status register returns to 0 (12ms)

    while ((spiReadRegMsk(dev, MAX7456ADD_STAT) & STAT_NVR_BUSY) != 0x00);

#ifdef USE_MS7456
    if (max7456DeviceType == MAX7456_DEVICE_TYPE_MS) {
        spiWriteReg(dev, 0x64, 0x9C);
        spiWriteReg(dev, 0x62, 0x02);
        if (!ms7456WaitForWriteProtect()) {
            spiWriteReg(dev, 0x5D, 0x02);
            return false;
        }
        spiWriteReg(dev, 0x5D, 0x02);
    }
#endif

    return true;
}

#ifdef MAX7456_NRST_PIN
static IO_t max7456ResetPin        = IO_NONE;
#endif

void max7456HardwareReset(void)
{
#ifdef MAX7456_NRST_PIN
#define IO_RESET_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)

    max7456ResetPin = IOGetByTag(IO_TAG(MAX7456_NRST_PIN));
    IOInit(max7456ResetPin, OWNER_OSD, 0);
    IOConfigGPIO(max7456ResetPin, IO_RESET_CFG);

    // RESET 50ms long pulse, followed by the device power-up delay.
    IOLo(max7456ResetPin);
    delay(50);
    IOHi(max7456ResetPin);
    delay(MAX7456_POWERUP_DELAY_MS);
#else
    // Allow the power supplies and 27MHz oscillator to stabilise before the
    // first SPI access. Some compatible devices need more than 50ms.
    delay(MAX7456_POWERUP_DELAY_MS);
#endif
}

bool max7456IsDeviceDetected(void)
{
    return max7456DeviceDetected;
}

void max7456SetBackgroundType(displayPortBackground_e backgroundType)
{
    deviceBackgroundType = backgroundType;

    max7456SetRegisterVM1();
}

#endif // USE_MAX7456
