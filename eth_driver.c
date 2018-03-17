#include "eth_driver.h"

uint8_t eth_driver_is_link_up(const eth_driver_t *const drv)
{
    return drv->is_link_up(drv);
}

uint8_t eth_driver_data_receive(const eth_driver_t *const drv,
                                uint8_t *const data,
                                uint8_t *const data_len)
{
    return drv->data_recv(drv, data, data_len);
}

uint8_t eth_driver_data_send(const eth_driver_t *const drv,
                             const uint8_t *const data,
                             const uint8_t data_len)
{
    return drv->data_send(drv, data, data_len);
}
