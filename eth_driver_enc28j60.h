#ifndef ETH_DRIVER_ENC_28J60_H
#define ETH_DRIVER_ENC_28J60_H

#include "bit_types.h"
#include "eth_driver.h"

uint8_t eth_driver_enc28j60_init(eth_driver_t *const drv,
                                 const uint8_t mac_address[6U]);

uint8_t eth_driver_enc28j60_is_link_up(const eth_driver_t *const drv);

uint8_t eth_driver_enc28j60_data_receive(const eth_driver_t *const drv,
                                         uint8_t *const data,
                                         uint16_t *const data_len);

uint8_t eth_driver_enc28j60_data_send(const eth_driver_t *const drv,
                                      const uint8_t *const data,
                                      const uint16_t data_len);

#endif
