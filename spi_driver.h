#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "bit_types.h"

enum spi_driver_return_codes_t
{
    SPI_RC_FAIL,
    SPI_RC_OK
};

typedef uint8_t (*data_send_ptr)(const uint8_t *const data_buf);
typedef uint8_t (*data_recv_ptr)(uint8_t **data_buf, uint8_t *len);

typedef
struct type_spi_driver
{
    data_send_ptr data_send;
    data_recv_ptr data_recv;

    // volatile uint8_t mutex;
} spi_driver_t;

uint8_t spi_driver_data_receive(const spi_driver_t *const drv,
                                uint8_t *const data,
                                uint8_t *const data_len);

uint8_t spi_driver_data_send(const spi_driver_t *const drv,
                             const uint8_t *const data,
                             const uint8_t data_len);

#endif

