#ifndef __RX_H_
#define __RX_H_

#include "stm32f0xx.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RX_SPI_FRSKY_D,
    RX_SPI_FRSKY_X,
    RX_SPI_SFHSS,
} rx_spi_protocol_e;

typedef enum {
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND = (1 << 0),
    RX_SPI_RECEIVED_DATA = (1 << 1),
    RX_SPI_ROCESSING_REQUIRED = (1 << 2),
} rx_spi_received_e;

// RC channels in AETR order
typedef enum {
    RC_SPI_ROLL = 0,
    RC_SPI_PITCH,
    RC_SPI_THROTTLE,
    RC_SPI_YAW,
    RC_SPI_AUX1,
    RC_SPI_AUX2,
    RC_SPI_AUX3,
    RC_SPI_AUX4,
    RC_SPI_AUX5,
    RC_SPI_AUX6,
    RC_SPI_AUX7,
    RC_SPI_AUX8,
    RC_SPI_AUX9,
    RC_SPI_AUX10,
    RC_SPI_AUX11,
    RC_SPI_AUX12,
    RC_SPI_AUX13,
    RC_SPI_AUX14
} rc_spi_aetr_e;

bool rxSpiSetProtocol(rx_spi_protocol_e protocol);
bool rxSpiGetExtiState(void);

#endif
