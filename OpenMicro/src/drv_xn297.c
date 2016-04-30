#include "binary.h"
#include "drv_spi.h"

#include "project.h"
#include "xn297.h"

void xn_writereg(uint8_t reg, uint8_t val) {
    reg = reg & 0x1F;
    reg = reg | 0x20;
    spi_cson();
    spi_sendbyte(reg);
    spi_sendbyte(val);
    spi_csoff();
}

uint8_t xn_readreg(uint8_t reg) {
    reg = reg & 0x1F;
    spi_cson();
    spi_sendrecvbyte(reg);
    int val = spi_sendzerorecvbyte();
    spi_csoff();
    return val;
}

uint8_t xn_command(uint8_t command) {
    spi_cson();
    int status = spi_sendrecvbyte(command);
    spi_csoff();
    return status;
}

void xn_readpayload(uint8_t *data, uint8_t size) {
    uint8_t index = 0;
    spi_cson();
    spi_sendrecvbyte( B01100001); // read rx payload
    while (index < size) {
        data[index] = spi_sendzerorecvbyte();
        index++;
    }
    spi_csoff();
}

void xn_writedata(uint8_t reg, uint8_t *addr, uint8_t size) {
    uint8_t index = 0;
    spi_cson();
    spi_sendbyte(reg | W_REGISTER);
    while (index < size) {
        spi_sendbyte(addr[index]);
        index++;
    }
    spi_csoff();
}

void xn_writerxaddress(uint8_t *addr) {
    xn_writedata(RX_ADDR_P0, addr, 5);
}

void xn_writetxaddress(uint8_t *addr) {
    xn_writedata(TX_ADDR, addr, 5);
}

void xn_writepayload(uint8_t data[], uint8_t size) {
    xn_writedata(W_TX_PAYLOAD, data, size);
}

