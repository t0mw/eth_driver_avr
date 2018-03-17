#include "spi_driver.h"

uint8_t spi_driver_data_receive(const spi_driver_t *const drv,
                                uint8_t *const data,
                                uint8_t *const data_len)
{
    return drv->data_recv(drv, data, data_len);
}

uint8_t spi_driver_data_send(const spi_driver_t *const drv,
                             const uint8_t *const data,
                             const uint8_t data_len)
{
    return drv->data_send(drv, data, data_len);
}

