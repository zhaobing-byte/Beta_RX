#ifndef __FRSKY_X_H_
#define __FRSKY_X_H_

#include "stm32f0xx.h"
#include <stdbool.h>
#include <stdint.h>
#include "rx.h"

#define SYNC_DELAY_MAX 9000
#define MAX_MISSING_PKT 100
enum {
    STATE_INIT = 0,
    STATE_BIND,
    STATE_BIND_TUNING,
    STATE_BIND_BINDING1,
    STATE_BIND_BINDING2,
    STATE_BIND_COMPLETE,
    STATE_STARTING,
    STATE_UPDATE,
    STATE_DATA,
    STATE_TELEMETRY,
    STATE_RESUME,
};
static bool getBind1(uint8_t *packet);
static bool getBind2(uint8_t *packet);
static bool tuneRx(uint8_t *packet);
rx_spi_received_e frSkySpiDataReceived(uint8_t *packet);
void frSkySpiSetRcData(uint16_t *rcData, const uint8_t *payload);
bool frSkySpiInit(void);
bool isValidPacket(const uint8_t *packet);
#endif


