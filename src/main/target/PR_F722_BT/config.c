/*
 * PR_F722-BT v1.5 target defaults
 */

#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "io/serial.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "target.h"

void targetConfiguration(void)
{
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioConfigMutable()->config[1] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;

    pinioBoxConfigMutable()->permanentId[0] = 40;
    pinioBoxConfigMutable()->permanentId[1] = 41;

    serialPortConfig_t *mspPort = serialFindPortConfigurationMutable(SERIAL_PORT_USART1);
    if (mspPort) {
        mspPort->functionMask = FUNCTION_MSP;
    }

    serialPortConfig_t *displayPort = serialFindPortConfigurationMutable(SERIAL_PORT_UART4);
    if (displayPort) {
        displayPort->functionMask = FUNCTION_MSP;
    }
}

#endif
