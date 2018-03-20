#include "arch_specific.h"
#include "enc28j60_defs.h"
#include "eth_driver_enc28j60.h"
#include "spi_driver_avr.h"

#include <string.h>

static spi_driver_t     spi_driver;
static uint16_t         next_packet_ptr = 0U;

#define ETH_RC_CHECK(x) \
            if ((x) != ETH_RC_OK) { \
                return ETH_RC_FAIL; \
            }

static uint8_t eth_driver_enc28j60_instruction_read(const uint8_t opcode,
                                                    const uint8_t arg,
                                                    uint8_t *const read_byte)
{
    uint8_t data[3] = { 0U };
    uint8_t data_len = 0U;

    data[0] = ( opcode | ( arg & ADDR_MASK ) );
    data[1] = 0x00U;
    data_len = 2U;

    // If this is an MAC/MII reg, ommit one dummy byte.
    if ((arg & 0x80U ) != 0U)
    {
        data[2] = 0x00U;
        data_len = 3U;
    }

    uint8_t rc_spi = 0U;
    rc_spi = spi_driver_data_send(&spi_driver,
                                  data,
                                  data_len);
    if (rc_spi != SPI_RC_OK)
        return ETH_RC_FAIL;

    data_len = 1U;
    rc_spi = spi_driver_data_receive(&spi_driver,
                                     data,
                                     &data_len);
    if (rc_spi != SPI_RC_OK)
        return ETH_RC_FAIL;

    *read_byte = data[0];

    return ETH_RC_OK;
}

static uint8_t eth_driver_enc28j60_instruction_write(const uint8_t opcode,
                                                     const uint8_t arg,
                                                     const uint8_t instr_data)
{
    uint8_t data[2] = { 0U };
    uint8_t data_len = 0U;

    data[0] = opcode | (arg & ADDR_MASK);
    data[1] = instr_data;
    data_len = 2U;

    uint8_t rc_spi = spi_driver_data_send(&spi_driver,
                                          data,
                                          data_len);
    if (rc_spi != SPI_RC_OK)
        return ETH_RC_FAIL;

    return ETH_RC_OK;
}

// SPI BFC, BFS commands.
static uint8_t eth_driver_enc28j60_bank_set(const uint8_t bank)
{
    static uint8_t current_bank = 0x00U;
    if ((bank & BANK_MASK) != current_bank)
    {
        uint8_t rc = 0U;
        rc = eth_driver_enc28j60_instruction_write(BFC,
                                                       ECON1,
                                                       ( BSEL0 | BSEL1 ));
        ETH_RC_CHECK(rc);

        rc = eth_driver_enc28j60_instruction_write(BFS,
                                                       ECON1,
                                                       ( bank & BANK_MASK ) >> 5U);
        ETH_RC_CHECK(rc);

        current_bank = (bank & BANK_MASK);
    }

    return ETH_RC_OK;
}

// SPI RCR command.
static uint8_t eth_driver_enc28j60_control_reg_read(const uint8_t reg_addr,
                                                    uint8_t *const read_byte)
{
    uint8_t rc = 0U;

    rc = eth_driver_enc28j60_bank_set(reg_addr);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_instruction_read(RCR,
                                              reg_addr,
                                              read_byte);
    ETH_RC_CHECK(rc);

    return ETH_RC_OK;
}

// SPI WCR command.
static uint8_t eth_driver_enc28j60_control_reg_write(const uint8_t reg_addr,
                                                     const uint8_t byte)
{
    uint8_t rc = 0U;

    rc = eth_driver_enc28j60_bank_set(reg_addr);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_instruction_write(WCR,
                                              reg_addr,
                                              byte);
    ETH_RC_CHECK(rc);

    return ETH_RC_OK;
}

// SPI phy register read.
static uint8_t eth_driver_enc28j60_phy_reg_read(const uint8_t addr,
                                                uint16_t *const read_byte)
{
    uint8_t rc = 0U;

    rc = eth_driver_enc28j60_control_reg_write(MIREGADR,
                                               addr);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(MICMD,
                                               MIIRD);
    ETH_RC_CHECK(rc);

    // _delay_us(15U);
    DELAY_WAIT_US(15U);

    uint8_t mistat_busy = BUSY;
    do
    {
        rc = eth_driver_enc28j60_control_reg_read(MISTAT,
                                                  &mistat_busy);
        ETH_RC_CHECK(rc);
    } while ((mistat_busy & BUSY ) != 0U);

    rc = eth_driver_enc28j60_control_reg_write(MICMD,
                                               0x00);
    ETH_RC_CHECK(rc);

    uint8_t byte = 0U;
    rc = eth_driver_enc28j60_control_reg_read(MIRDL,
                                              &byte);
    ETH_RC_CHECK(rc);

    *read_byte = byte;

    rc = eth_driver_enc28j60_control_reg_read(MIRDH,
                                              &byte);
    ETH_RC_CHECK(rc);

    *read_byte |= ( ( (uint16_t)byte ) << 8U );

    return ETH_RC_OK;
}

// SPI phy register write.
static uint8_t eth_driver_enc28j60_phy_reg_write(const uint8_t addr,
                                                 const uint16_t data)
{
    uint8_t rc = 0U;

    rc = eth_driver_enc28j60_control_reg_write(MIREGADR,
                                               addr);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(MIWRL,
                                               ( data & 0xffU ));
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(MIWRH,
                                               (( data & 0xff00U ) >> 8U ));
    ETH_RC_CHECK(rc);

    uint8_t mistat_busy = BUSY;

    do
    {
        rc = eth_driver_enc28j60_control_reg_read(MISTAT,
                                                  &mistat_busy);
        ETH_RC_CHECK(rc);
    } while(( mistat_busy & BUSY) != 0U);

    return ETH_RC_OK;
}

// Send ethernet packet.
static uint8_t eth_driver_enc28j60_eth_packet_send(const uint8_t *const packet,
                                                   const uint16_t packet_len)
{
    uint8_t rc = 0U;

    uint8_t ctrl_reg_val = 0U;
    rc = eth_driver_enc28j60_control_reg_read(ECON1,
                                              &ctrl_reg_val);
    ETH_RC_CHECK(rc);

    while (( ctrl_reg_val & TXRTS ) != 0U)
    {
        rc = eth_driver_enc28j60_control_reg_read(EIR,
                                                  &ctrl_reg_val);
        ETH_RC_CHECK(rc);

        if ((ctrl_reg_val & TXERIF ) != 0U)
        {
            rc = eth_driver_enc28j60_instruction_write(BFS,
                                                       ECON1,
                                                       TXRST);
            ETH_RC_CHECK(rc);

            rc = eth_driver_enc28j60_instruction_write(BFS,
                                                       ECON1,
                                                       TXRST);
            ETH_RC_CHECK(rc);
        }

        rc = eth_driver_enc28j60_control_reg_read(ECON1,
                                                  &ctrl_reg_val);
        ETH_RC_CHECK(rc);
    }

    // Setup enc28j60 pointers.
    // Currently only the start of page is written.
    rc = eth_driver_enc28j60_control_reg_write(EWRPTL,
                                               ( TXSTART_INIT & 0xFFU ));
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(EWRPTH,
                                               ( TXSTART_INIT & 0xFF00U ) >> 8U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ETXNDL,
                                               ( TXSTART_INIT & 0xFFU ));
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ETXNDH,
                                               ( TXSTART_INIT & 0xFFU ) >> 8U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_instruction_write(WBM,
                                               0U,
                                               0U);
    ETH_RC_CHECK(rc);

    // Send the actual packet.
    uint8_t rc_spi = spi_driver_data_send(&spi_driver,
                                          packet,
                                          packet_len);
    if (rc_spi != SPI_RC_OK)
        return ETH_RC_FAIL;

    rc = eth_driver_enc28j60_instruction_write(BFS,
                                               ECON1,
                                               TXRTS);
    ETH_RC_CHECK(rc);

    return ETH_RC_OK;
}

// Receive ethernet packet.
static uint8_t eth_driver_enc28j60_eth_packet_receive(uint8_t *const packet,
                                                      uint16_t *const packet_len)
{
    uint8_t rc = 0U;

    uint8_t packet_count = 0U;
    rc = eth_driver_enc28j60_control_reg_read(EPKTCNT,
                                              &packet_count);
    ETH_RC_CHECK(rc);

    if (packet_count == 0U)
        return ETH_RC_ETH_PACKET_RECV_NO_DATA;

    // Setup packet pointers.
    rc = eth_driver_enc28j60_control_reg_write(ERDPTL,
                                               next_packet_ptr & 0xffU);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ERDPTH,
                                               ( next_packet_ptr & 0xff00U ) >> 8U);
    ETH_RC_CHECK(rc);

    // Read the location of next packet pointer.
    uint8_t tmp_next_packet_ptr = 0U;
    rc = eth_driver_enc28j60_instruction_read(RBM,
                                              0U,
                                              &tmp_next_packet_ptr);
    ETH_RC_CHECK(rc);

    next_packet_ptr = tmp_next_packet_ptr;

    rc = eth_driver_enc28j60_instruction_read(RBM,
                                              0U,
                                              &tmp_next_packet_ptr);
    ETH_RC_CHECK(rc);

    next_packet_ptr |= ( (uint16_t)tmp_next_packet_ptr ) << 8U;

    // Read the length of received packet.
    uint8_t recvd_packet_len = 0U;
    rc = eth_driver_enc28j60_instruction_read(RBM,
                                              0U,
                                              &recvd_packet_len);
    ETH_RC_CHECK(rc);

    *packet_len = recvd_packet_len;

    rc = eth_driver_enc28j60_instruction_read(RBM,
                                              0U,
                                              &recvd_packet_len);
    ETH_RC_CHECK(rc);

    *packet_len |= ( (uint16_t)recvd_packet_len ) << 8U;
    *packet_len -= 4;

    // Read RX Stats.
    uint8_t rx_stats_byte = 0U;
    uint16_t rx_stats = 0U;
    rc = eth_driver_enc28j60_instruction_read(RBM,
                                              0U,
                                              &rx_stats_byte);
    ETH_RC_CHECK(rc);

    rx_stats = rx_stats_byte;

    rc = eth_driver_enc28j60_instruction_read(RBM,
                                              0U,
                                              &rx_stats_byte);
    ETH_RC_CHECK(rc);

    rx_stats |= ( (uint16_t)rx_stats_byte ) << 8U;

    // Check if RECEIVED_OK is set.
    if (( rx_stats & 0x80 ) == 0U)
        return ETH_RC_FAIL;

    // Read the data.
    uint16_t packet_len_to_process = *packet_len;
    uint8_t data_byte = 0U;
    uint8_t *packet_ptr = packet;
    while (packet_len_to_process != 0)
    {
        rc = eth_driver_enc28j60_instruction_read(RBM,
                                                  0U,
                                                  &data_byte);
        ETH_RC_CHECK(rc);

        *packet_ptr = data_byte;
        ++packet_ptr;

        --packet_len_to_process;
    }

/*
    rc = eth_driver_enc28j60_control_reg_write(ERXRDPTL,
                                               next_packet_ptr & 0xFFU);
    if (rc != ETH_RC_CONTROL_REG_WRITE_OK)
        return ETH_RC_ETH_PACKET_RECV_FAIL;

    rc = eth_driver_enc28j60_control_reg_write(ERXRDPTH,
                                               ( next_packet_ptr & 0xFF00U ) >> 8U);
    if (rc != ETH_RC_CONTROL_REG_WRITE_OK)
        return ETH_RC_ETH_PACKET_RECV_FAIL;
*/

    // ????? unsigned & 0 comparison
    // if (( next_packet_ptr - 1U ) < RXSTART_INIT ||
    //    ( next_packet_ptr - 1U ) > RXSTOP_INIT)

    if (( next_packet_ptr - 1U ) > RXSTOP_INIT)
    {
        rc = eth_driver_enc28j60_control_reg_write(ERXRDPTL,
                                                   ( RXSTOP_INIT ) & 0xFFU);
        ETH_RC_CHECK(rc);

        rc = eth_driver_enc28j60_control_reg_write(ERXRDPTH,
                                                   ( ( RXSTOP_INIT ) & 0xFF00U ) >> 8U);
        ETH_RC_CHECK(rc);
    }
    else
    {
        rc = eth_driver_enc28j60_control_reg_write(ERXRDPTL,
                                                   ( next_packet_ptr - 1U ) & 0xFFU);
        ETH_RC_CHECK(rc);

        rc = eth_driver_enc28j60_control_reg_write(ERXRDPTH,
                                                   ( ( next_packet_ptr - 1U ) & 0xFF00U ) >> 8U);
        ETH_RC_CHECK(rc);
    }

    // Decrement the packet counter.
    rc = eth_driver_enc28j60_instruction_write(BFS,
                                               ECON2,
                                               PKTDEC);
    ETH_RC_CHECK(rc);

    return ETH_RC_OK;
}

static uint8_t eth_driver_enc28j60_hardware_init(const eth_driver_t *const drv)
{
    uint8_t rc = 0U;

    rc = eth_driver_enc28j60_instruction_write(SC,
                                               0,
                                               SC);

    // _delay_us(205); // ???
    DELAY_WAIT_US(205U);

    next_packet_ptr = RXSTART_INIT;

    rc = eth_driver_enc28j60_control_reg_write(ERXSTL,
                                               RXSTART_INIT & 0xFFU);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ERXSTH,
                                               ( RXSTART_INIT & 0xFF00U ) >> 8U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ERXRDPTL,
                                               RXSTART_INIT & 0xFFU);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(ERXRDPTH,
                                               ( RXSTART_INIT & 0xFF00U ) >> 8U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ERXNDL,
                                               RXSTOP_INIT & 0xFFU);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(ERXNDH,
                                              ( RXSTOP_INIT & 0xFF00U ) >> 8U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ETXSTL,
                                               TXSTART_INIT & 0xFFU);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(ETXSTH,
                                               ( TXSTART_INIT & 0xFF00U ) >> 8U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ETXNDL,
                                               TXSTOP_INIT & 0xFFU);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(ETXNDH,
                                               ( TXSTOP_INIT & 0xFF00U ) >> 8U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(ERXFCON,
                                               ( UCEN | CRCEN | PMEN ));
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(EPMM0,
                                               0x3fU);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(EPMM1,
                                               0x30U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(EPMCSL,
                                               0xf9U);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(EPMCSH,
                                               0xf7U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(MACON1,
                                               ( MARXEN | TXPAUS | RXPAUS ));
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MACON2,
                                               0x00U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_instruction_write(BFS,
                                               MACON3,
                                               ( PADCFG0 | TXCRCEN | FRMLNEN));
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(MAIPGL,
                                               0x12U);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MAIPGH,
                                               0x0CU);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(MABBIPG,
                                               0x12U);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_control_reg_write(MAMXFLL,
                                               ( MAX_FRAMELEN & 0xFFU ));
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MAMXFLH,
                                               ( MAX_FRAMELEN & 0xFF00U ) >> 8U);
    ETH_RC_CHECK(rc);

    // Set MAC address.
    rc = eth_driver_enc28j60_control_reg_write(MAADR5,
                                               drv->mac_address[0]);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MAADR4,
                                               drv->mac_address[1]);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MAADR3,
                                               drv->mac_address[2]);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MAADR2,
                                               drv->mac_address[3]);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MAADR1,
                                               drv->mac_address[4]);
    ETH_RC_CHECK(rc);
    rc = eth_driver_enc28j60_control_reg_write(MAADR0,
                                               drv->mac_address[5]);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_phy_reg_write(PHCON2,
                                           HDLDIS);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_bank_set(ECON1);
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_instruction_write(BFS,
                                               EIE,
                                               ( INTIE | PKTIE ));
    ETH_RC_CHECK(rc);

    rc = eth_driver_enc28j60_instruction_write(BFS,
                                               ECON1,
                                               RXEN);
    ETH_RC_CHECK(rc);

    return ETH_RC_OK;
}

uint8_t eth_driver_enc28j60_init(eth_driver_t *const drv,
                                 const uint8_t *const mac_address)
{
    memcpy(drv->mac_address,
           mac_address,
           6U);

    drv->is_link_up = eth_driver_enc28j60_is_link_up;
    drv->data_recv = eth_driver_enc28j60_data_receive;
    drv->data_send = eth_driver_enc28j60_data_send;

    // Initialize SPI driver.
    uint8_t rc_spi_init = spi_driver_avr_init(&spi_driver);
    if (rc_spi_init == SPI_RC_FAIL)
        return ETH_RC_FAIL;

    return eth_driver_enc28j60_hardware_init(drv);
}

uint8_t eth_driver_enc28j60_is_link_up(const eth_driver_t *const drv)
{
    (void)drv;
    return ETH_RC_OK;
}

uint8_t eth_driver_enc28j60_data_receive(const eth_driver_t *const drv,
                                         uint8_t *const data,
                                         uint16_t *const data_len)
{
    uint8_t eth_frame_buffer_recv[ETH_FRAME_SIZE_MAX];
    uint8_t eth_frame_buffer_recv_size = 0U;

    // Receive eth frame from enc28j60.
    uint8_t rc = eth_driver_enc28j60_eth_packet_receive(data,
                                                        data_len);
    ETH_RC_CHECK(rc);

    memcpy(data,
           eth_frame_buffer_recv,
           eth_frame_buffer_recv_size);

    *data_len = eth_frame_buffer_recv_size;

    return ETH_RC_OK;
}

uint8_t eth_driver_enc28j60_data_send(const eth_driver_t *const drv,
                                      const uint8_t *const data,
                                      const uint16_t data_len)
{
    // TODO: need to encapsulate data into ethernet
    //       frame header.

    // Send eth frame into enc28j60.
    uint8_t rc = eth_driver_enc28j60_eth_packet_send(data,
                                                     data_len);
    ETH_RC_CHECK(rc);

    return ETH_RC_OK;
}
