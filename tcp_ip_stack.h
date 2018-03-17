#ifndef TCP_IP_STACK_H
#define TCP_IP_STACK_H

#include "bit_types.h"
#include "eth_driver.h"

#define TCP_IP_IP_FRAME_HEADER_SIZE     20U

enum tcp_ip_return_codes_t
{
    TCP_IP_RC_FAIL,
    TCP_IP_RC_OK,
    TCP_IP_RC_IP_DIFFERENT_DESTINATION
};

typedef
enum type_tcp_ip_protocol
{
    TCP_IP_ETHERNET,

    TCP_IP_ARP,
    TCP_IP_IP,

    TCP_IP_TCP,
    TCP_IP_UDP
} tcp_ip_protocol_t;

typedef void (*tcp_ip_hook_recv)(const tcp_ip_stack_t *const tcp_ip_stack,
                                 const uint8_t *const data,
                                 const uint8_t data_len,
                                 uint8_t *const response_data,
                                 uint8_t *const respose_data_len);

typedef
struct type_tcp_ip_stack_bind_data
{
    // TODO: bind option could be a "bitfield" and
    //       tell which hooks to notify.
    tcp_ip_protocol_t bind_option;
    uint8_t               bind_port;
    tcp_ip_hook_recv      hook_recv;
} tcp_ip_bind_data_t;

typedef
struct type_tcp_ip_stack
{
    eth_driver_t *eth_driver;
    uint8_t ipv4_address[4U];

    tcp_ip_bind_data_t bind_data;

    // volatile uint8_t mutex;
} tcp_ip_stack_t;

uint8_t tcp_ip_initialize(tcp_ip_stack_t *const stack,
                          const eth_driver_t *const drv,
                          const uint8_t *const ipv4_addr);

uint8_t tcp_ip_make_frame_ip(tcp_ip_stack_t *const stack,
                             const uint8_t *const data,
                             const uint8_t data_len,
                             uint8_t *const frame,
                             uint8_t *const frame_len);

uint8_t tcp_ip_make_frame_tcp(tcp_ip_stack_t *const stack,
                              const uint8_t *const data,
                              const uint8_t data_len,
                              uint8_t *const frame,
                              uint8_t *const frame_len);

uint8_t tcp_ip_make_frame_udp(tcp_ip_stack_t *const stack,
                              const uint8_t *const data,
                              const uint8_t data_len,
                              uint8_t *const frame,
                              uint8_t *const frame_len);

uint8_t tcp_ip_send_tcp(tcp_ip_stack_t *const stack,
                        const uint8_t *const data,
                        const uint8_t data_len);

uint8_t tcp_ip_send_udp(tcp_ip_stack_t *const stack,
                        const uint8_t *const data,
                        const uint8_t data_len);

uint8_t tcp_ip_bind(tcp_ip_stack_t *const stack,
                    const tcp_ip_protocol_t bind_option,
                    const uint8_t bind_port,
                    tcp_ip_hook_recv hook_recv);

uint8_t tcp_ip_process_step(const tcp_ip_stack_t *const stack);

uint8_t tcp_ip_process(const tcp_ip_stack_t *const stack);

#endif
