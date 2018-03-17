#ifndef SPI_DRIVER_AVR_H
#define SPI_DRIVER_AVR_H

#include "bit_types.h"
#include "spi_driver.h"

uint8_t spi_driver_avr_init(spi_driver_t *const drv);

uint8_t spi_driver_avr_data_receive(const spi_driver_t *const drv,
                                    uint8_t *const data,
                                    uint8_t *const data_len);

uint8_t spi_driver_avr_data_send(const spi_driver_t *const drv,
                                 const uint8_t *const data,
                                 const uint8_t data_len);

#endif

