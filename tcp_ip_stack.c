#include "tcp_ip_stack.h"

#include <string.h>

#define TCP_IP_RC_CHECK(x) \
            if ((x) != TCP_IP_RC_OK) { \
                return TCP_IP_RC_FAIL; \
            }

#define TCP_IP_IP_DESTINATION   0x1eU
#define TCP_IP_IP_SOURCE        0x1aU

static tcp_ip_protocol_t tcp_ip_frame_eth_get_protocol(const uint8_t *const frame)
{
    static const uint8_t ETHERTYPE_POS      = 0x0CU;
    static const uint8_t ETHERTYPE_ARP_H    = 0x08U;
    static const uint8_t ETHERTYPE_ARP_L    = 0x06U;
    static const uint8_t ETHERTYPE_IP_H     = 0x08U;
    static const uint8_t ETHERTYPE_IP_L     = 0x00U;

    const uint8_t ethertype_high = (uint8_t)frame[ETHERTYPE_POS];
    const uint8_t ethertype_low = (uint8_t)frame[ETHERTYPE_POS + 1];

    if(ethertype_high == ETHERTYPE_IP_H &&
       ethertype_low == ETHERTYPE_ARP_L)
       return TCP_IP_PROTO_ARP;

    if(ethertype_high == ETHERTYPE_ARP_H &&
       ethertype_low == ETHERTYPE_IP_L)
       return TCP_IP_PROTO_IP;

    return TCP_IP_PROTO_UNKNOWN;
}

static tcp_ip_protocol_t tcp_ip_frame_ip_get_protocol(const uint8_t *const frame)
{
    // return TCP/UDP

    static const uint8_t IP_PROTOCOL_POS    = 0x09U;
    static const uint8_t IP_PROTOCOL_TCP    = 0x06U;
    static const uint8_t IP_PROTOCOL_UDP    = 0x11U;
    static const uint8_t IP_PROTOCOL_ICMP   = 0x01U;

    const uint8_t ip_protocol = frame[IP_PROTOCOL_POS];

    if(ip_protocol == IP_PROTOCOL_TCP)
       return TCP_IP_PROTO_TCP;

    if(ip_protocol == IP_PROTOCOL_UDP)
       return TCP_IP_PROTO_UDP;

    if(ip_protocol == IP_PROTOCOL_ICMP)
       return TCP_IP_PROTO_ICMP;

    return TCP_IP_PROTO_UNKNOWN;
}

static void tcp_ip_hooks_notify(const tcp_ip_stack_t *const stack,
                                const tcp_ip_protocol_t protocol,
                                const uint8_t *const frame,
                                const uint16_t frame_len,
                                uint8_t *const response_data,
                                uint16_t *const response_data_len)
{
    // Check if binded to "protocol".

    // Send only the "data" inside of protocol in "frame".
    //  i.e. binded to IP - send only TCP frame, binded
    //  to TCP - send only data. Cut down reponse_data
    // to point into "data" part of the protocol.
}

static uint8_t tcp_ip_handle_arp(const tcp_ip_stack_t *const stack,
                                 const uint8_t *const frame,
                                 const uint16_t frame_len,
                                 uint8_t *const frame_response,
                                 uint16_t *const frame_response_len)
{
    // ARP reply.

    return TCP_IP_RC_OK;
}

static uint8_t tcp_ip_handle_tcp(const tcp_ip_stack_t *const stack,
                                 const uint8_t *const frame,
                                 const uint16_t frame_len,
                                 uint8_t *const frame_response,
                                 uint16_t *const frame_response_len)
{
    // Adjust seq/ack numbers.
    // Send reply.

    return TCP_IP_RC_OK;
}

static uint8_t tcp_ip_handle_udp(const tcp_ip_stack_t *const stack,
                                 const uint8_t *const frame,
                                 const uint16_t frame_len,
                                 uint8_t *const frame_response,
                                 uint16_t *const frame_response_len)
{
    // Verify that there is a response for this packet.
    if (frame_response_len == 0U)
        return TCP_IP_RC_OK; // Nothing to do.

    uint8_t rc = 0U;

    // frame_response is a full ethernet-sized frame, with
    // "data" field containing some data from user. Now add
    // the UDP header.

    // TODO:
    //      Send the response here via tcp_ip_send_udp ?

    rc = tcp_ip_make_frame_udp(stack,
                               frame_response + TCP_IP_IP_FRAME_HEADER_SIZE + TCP_IP_UDP_FRAME_HEADER_SIZE,
                               (*frame_response_len) - TCP_IP_IP_FRAME_HEADER_SIZE - TCP_IP_UDP_FRAME_HEADER_SIZE,
                               frame_response,
                               frame_response_len);
    TCP_IP_RC_CHECK(rc);

    return TCP_IP_RC_OK;
}

static uint8_t tcp_ip_handle_ip(const tcp_ip_stack_t *const stack,
                                const uint8_t *const frame,
                                const uint16_t frame_len,
                                uint8_t *const frame_response,
                                uint16_t *const frame_response_len)
{
    // Is this packet addressed for this device?
    uint8_t i = 0U;
    for (i = 0U; i < 4; ++i)
    {
        if (frame[TCP_IP_IP_DESTINATION + i] != stack->ipv4_address[i])
            return TCP_IP_RC_IP_DIFFERENT_DESTINATION;
    }

    uint8_t rc = 0U;

    tcp_ip_protocol_t protocol = tcp_ip_frame_ip_get_protocol(frame);

    switch (protocol)
    {
        case TCP_IP_PROTO_TCP:
        {
            // If not established, do three way handshake.

            // Notify hooks registered for IP-encapsulated frames only
            // if the frame contained actual data - no need to spam hooks
            // with three-way handshake packets.
            tcp_ip_hooks_notify(stack,
                                protocol,
                                frame,
                                frame_len,
                                frame_response,
                                frame_response_len);

            rc = tcp_ip_handle_tcp(stack,
                                   frame,
                                   frame_len,
                                   frame_response,
                                   frame_response_len);
        }
        break;

        case TCP_IP_PROTO_UDP:
        {
            // Notify hooks registered for IP-encapsulated frames.
            tcp_ip_hooks_notify(stack,
                                protocol,
                                frame,
                                frame_len,
                                frame_response,
                                frame_response_len);

            rc = tcp_ip_handle_udp(stack,
                                   frame,
                                   frame_len,
                                   frame_response,
                                   frame_response_len);

        }
        break;

        default:
        break;
    }

    // Now pack the data with IP header if there is a response
    // to send.
    if (*frame_response_len == 0U)
        return TCP_IP_RC_OK; // Nothing to do.

    rc = tcp_ip_make_frame_ip(stack,
                              frame_response + TCP_IP_IP_FRAME_HEADER_SIZE,
                              (*frame_response_len) - TCP_IP_IP_FRAME_HEADER_SIZE,
                              frame_response,
                              frame_response_len);
    TCP_IP_RC_CHECK(rc);

    return rc;
}

uint8_t tcp_ip_initialize(tcp_ip_stack_t *const stack,
                          eth_driver_t *const drv,
                          const uint8_t *const ipv4_addr)
{
    memcpy(stack->ipv4_address,
           ipv4_addr,
           4U);

    stack->eth_driver = drv;

    return TCP_IP_RC_OK;
}

uint8_t tcp_ip_make_frame_ip(const tcp_ip_stack_t *const stack,
                             const uint8_t *const data,
                             const uint16_t data_len,
                             uint8_t *const frame,
                             uint16_t *const frame_len)
{
    // ...

    return TCP_IP_RC_OK;
}

uint8_t tcp_ip_make_frame_tcp(const tcp_ip_stack_t *const stack,
                              const uint8_t *const data,
                              const uint16_t data_len,
                              uint8_t *const frame,
                              uint16_t *const frame_len)
{
    // ...

    return TCP_IP_RC_OK;
}

uint8_t tcp_ip_make_frame_udp(const tcp_ip_stack_t *const stack,
                              const uint8_t *const data,
                              const uint16_t data_len,
                              uint8_t *const frame,
                              uint16_t *const frame_len)
{
    // ...

    return TCP_IP_RC_OK;
}

uint8_t tcp_ip_send_tcp(tcp_ip_stack_t *const stack,
                        const uint8_t *const data,
                        const uint16_t data_len)
{
    uint8_t rc = 0U;

    // Fragmentation not allowed - always send an IP packet that can be
    // fit into ethernet packet.
    uint8_t frame[ETH_FRAME_SIZE_MAX - ETH_FRAME_HEADER_SIZE_NO_CHECKSUM] = { 0U };
    uint16_t frame_len = 0U;

    // TODO:
    //      Is the session established?
    //      Do we need to do a three-way-handshake?
    //      Are the SEQ ACK in sync?

    // Add TCP header to data.
    rc = tcp_ip_make_frame_tcp(stack,
                               data,
                               data_len,
                               frame + TCP_IP_IP_FRAME_HEADER_SIZE,
                               &frame_len);
    TCP_IP_RC_CHECK(rc);

    // Add IP header to TCP frame.
    rc = tcp_ip_make_frame_ip(stack,
                              frame + TCP_IP_IP_FRAME_HEADER_SIZE,
                              frame_len,
                              frame,
                              &frame_len);
    TCP_IP_RC_CHECK(rc);

    // Send the IP frame over ethernet.
    rc = eth_driver_data_send(stack->eth_driver,
                              frame,
                              frame_len);
    TCP_IP_RC_CHECK(rc);

    return TCP_IP_RC_OK;
}

uint8_t tcp_ip_send_udp(tcp_ip_stack_t *const stack,
                        const uint8_t *const data,
                        const uint16_t data_len)
{
    uint8_t rc = 0U;

    // Format UDP frame, send over ethernet.
    uint8_t frame[ETH_FRAME_SIZE_MAX - ETH_FRAME_HEADER_SIZE_NO_CHECKSUM] = { 0U };
    uint16_t frame_len = 0U;

    rc = tcp_ip_make_frame_udp(stack,
                               data,
                               data_len,
                               frame + TCP_IP_IP_FRAME_HEADER_SIZE,
                               &frame_len);
    TCP_IP_RC_CHECK(rc);

    rc = tcp_ip_make_frame_ip(stack,
                              data,
                              data_len,
                              frame,
                              &frame_len);
    TCP_IP_RC_CHECK(rc);

    // Send the IP frame over ethernet.
    rc = eth_driver_data_send(stack->eth_driver,
                              frame,
                              frame_len);
    TCP_IP_RC_CHECK(rc);

    return TCP_IP_RC_OK;
}

uint8_t tcp_ip_bind(tcp_ip_stack_t *const stack,
                    const tcp_ip_protocol_t bind_option,
                    const uint16_t bind_port,
                    tcp_ip_hook_recv hook_recv)
{
    stack->bind_data.bind_option = bind_option;
    stack->bind_data.bind_port = bind_port;
    stack->bind_data.hook_recv = hook_recv;

    return TCP_IP_RC_OK;
}

uint8_t tcp_ip_process_step(const tcp_ip_stack_t *const stack)
{
    uint8_t rc = 0U;

    uint8_t frame[ETH_FRAME_SIZE_MAX - ETH_FRAME_HEADER_SIZE_NO_CHECKSUM] = { 0U };
    uint16_t frame_len = 0U;
    rc = eth_driver_data_receive(stack->eth_driver,
                                 frame,
                                 &frame_len);
    TCP_IP_RC_CHECK(rc);

    uint8_t frame_response[ETH_FRAME_SIZE_MAX - ETH_FRAME_HEADER_SIZE_NO_CHECKSUM] = { 0U };
    uint16_t frame_response_len = 0U;

    tcp_ip_protocol_t protocol = tcp_ip_frame_eth_get_protocol(frame);

    // Notify hooks registered for ethernet-encapsulated frames.
    tcp_ip_hooks_notify(stack,
                        protocol,
                        frame,
                        frame_len,
                        frame_response,
                        &frame_response_len);

    switch (protocol)
    {
        case TCP_IP_PROTO_ARP:
            rc = tcp_ip_handle_arp(stack,
                                   frame + TCP_IP_ARP_FRAME_HEADER_SIZE,
                                   frame_len - TCP_IP_ARP_FRAME_HEADER_SIZE,
                                   frame_response,
                                   &frame_response_len);
        break;

        case TCP_IP_PROTO_IP:
            rc = tcp_ip_handle_ip(stack,
                                  frame + ETH_FRAME_HEADER_SIZE_NO_CHECKSUM,
                                  frame_len - ETH_FRAME_HEADER_SIZE_NO_CHECKSUM,
                                  frame_response,
                                  &frame_response_len);
        break;

        default:
        break;
    }

    return rc;
}

uint8_t tcp_ip_process(const tcp_ip_stack_t *const stack)
{
    uint8_t rc = TCP_IP_RC_OK;

    while (rc == TCP_IP_RC_OK)
    {
        rc = tcp_ip_process_step(stack);
    }

    return rc;
}
