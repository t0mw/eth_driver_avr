#ifndef ETH_DRIVER_H
#define ETH_DRIVER_H

#include "bit_types.h"
#include "enc28j60_defs.h"

#define ETH_FRAME_SIZE_MAX                  1522U
#define ETH_FRAME_HEADER_SIZE_NO_CHECKSUM   14U
#define ETH_FRAME_HEADER_SIZE               18U

enum eth_driver_return_codes_t
{
    ETH_RC_FAIL,
    ETH_RC_OK,
    ETH_RC_ETH_PACKET_RECV_NO_DATA
};

struct type_eth_driver;
typedef struct type_eth_driver eth_driver_t;

typedef uint8_t (*eth_driver_link_up_ptr)(const eth_driver_t *const drv);
typedef uint8_t (*eth_driver_data_send_ptr)(const eth_driver_t *const drv,
                                            const uint8_t *const data_buf,
                                            const uint16_t data_buf_len);
typedef uint8_t (*eth_driver_data_recv_ptr)(const eth_driver_t *const drv,
                                            uint8_t *const data_buf,
                                            uint16_t *const data_buf_len);

typedef
struct type_eth_driver
{
    eth_driver_link_up_ptr   is_link_up;
    eth_driver_data_send_ptr data_send;
    eth_driver_data_recv_ptr data_recv;

    uint8_t       mac_address[6U];

    // volatile uint8_t mutex;
} eth_driver_t;

uint8_t eth_driver_is_link_up(const eth_driver_t *const drv);

uint8_t eth_driver_data_receive(const eth_driver_t *const drv,
                                uint8_t *const data,
                                uint16_t *const data_len);

uint8_t eth_driver_data_send(const eth_driver_t *const drv,
                             const uint8_t *const data,
                             const uint16_t data_len);

#endif

