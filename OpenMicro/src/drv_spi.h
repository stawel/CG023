#ifndef DRV_SPI_H
#define DRV_SPI_H

// soft spi  header file 
//
#include <stdint.h>

void spi_init(void);
void spi_cson(void);
void spi_csoff(void);
void spi_sendbyte(uint8_t);
uint8_t spi_sendrecvbyte(uint8_t);
uint8_t spi_sendzerorecvbyte(void);

#endif
