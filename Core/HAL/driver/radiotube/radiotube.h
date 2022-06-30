#ifndef _RADIOTUBE_H
#define _RADIOTUBE_H

#include "bsp_spi.h"
#include "monitor.h"
#include "cvector.h"

typedef struct _radiotube_config_t {
    uint8_t bsp_spi_index; 
} radiotube_config;

typedef struct radiotube_t {
    uint8_t data;
    radiotube_config config;
} radiotube;

typedef struct Spi_io_o{
    int tx_port;
    int rx_port;
}Spi_io;

void Radiotube_Init(void);
void Radiotube_Transmit(void);
void Radiotube_Receive(radiotube* obj,uint8_t *radiotube_rx_data);
radiotube* Radiotube_Create(radiotube_config* config);

#endif
