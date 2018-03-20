/*
     Sample file for enc28j60 driver.
*/

#include "arch_specific.h"
#include "bit_types.h"
#include "eth_driver.h"
#include "eth_driver_enc28j60.h"
#include "tcp_ip_stack.h"

#include <string.h>

void recv_udp_5050_hook(const tcp_ip_stack_t *const tcp_ip_stack,
                      const uint8_t *const data,
                      const uint16_t data_len,
                      uint8_t *const response_data,
                      uint16_t *const respose_data_len)
{
    // Received some data over UDP:5050, respond.
}

int main(void)
{
    const uint8_t mac_address[] = { 0xabU, 0xabU, 0xabU, 0xabU, 0xabU, 0xabU };
    const uint8_t ip_address[] = { 192U, 168U, 1U, 54U };

    eth_driver_t eth_driver = { 0U };

    uint8_t rc = 0U;

    // Initialize specific driver and fill the proxy structure.
    rc = eth_driver_enc28j60_init(&eth_driver,
                                  mac_address);
    p_assert(rc == ETH_RC_OK);

    tcp_ip_stack_t tcp_ip_stack = { 0U };

    rc = tcp_ip_initialize(&tcp_ip_stack,
                           &eth_driver,
                           ip_address);
    p_assert(rc == TCP_IP_RC_OK);

    rc = tcp_ip_bind(&tcp_ip_stack,
                     TCP_IP_PROTO_UDP,
                     5050U,
                     recv_udp_5050_hook);
    p_assert(rc == TCP_IP_RC_OK);

    const uint8_t still_running = 1U;
    while (still_running > 0U)
    {
        rc = tcp_ip_process_step(&tcp_ip_stack);

        if (rc != TCP_IP_RC_OK)
            continue; // do stuff
    }

    return 0;
}
