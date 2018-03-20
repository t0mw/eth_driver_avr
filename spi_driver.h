#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "bit_types.h"

enum spi_driver_return_codes_t
{
    SPI_RC_FAIL,
    SPI_RC_OK
};

struct type_spi_driver;
typedef struct type_spi_driver spi_driver_t;

typedef uint8_t (*spi_data_send_ptr)(const spi_driver_t *const drv,
                                     const uint8_t *const data_buf,
                                     const uint8_t data_buf_len);
typedef uint8_t (*spi_data_recv_ptr)(const spi_driver_t *const drv,
                                     uint8_t *const data_buf,
                                     uint8_t *const len);

typedef
struct type_spi_driver
{
    spi_data_send_ptr data_send;
    spi_data_recv_ptr data_recv;

    // volatile uint8_t mutex;
} spi_driver_t;

uint8_t spi_driver_data_receive(const spi_driver_t *const drv,
                                uint8_t *const data,
                                uint8_t *const data_len);

uint8_t spi_driver_data_send(const spi_driver_t *const drv,
                             const uint8_t *const data,
                             const uint8_t data_len);

#endif

