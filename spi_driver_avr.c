#include "spi_driver_avr.h"

// Assume there's only one SPI interface for this
// AVR - if mutex = 0, can access. Else, busy.
// static volatile uint8_t spi_driver_avr_mutex = 0;

uint8_t spi_driver_avr_init(spi_driver_t *const drv)
{
    // TODO

    return SPI_RC_OK;
}

uint8_t spi_driver_avr_data_receive(const spi_driver_t *const drv,
                                    uint8_t *const data,
                                    uint8_t *const data_len)
{
    // TODO

    return SPI_RC_OK;
}

uint8_t spi_driver_avr_data_send(const spi_driver_t *const drv,
                                 const uint8_t *const data,
                                 const uint8_t data_len)
{
    // TODO

    return SPI_RC_OK;
}
