#include "project.h"
#include "xn297.h"
#include "binary.h"

#define XN_DEBUG_BUFFER 256
#define XN_DEBUG_PACKAGE 16

static const uint8_t txaddress[5] = {0xcc, 0xcc, 0xcc, 0xcc, 0xcc};
static uint8_t data[XN_DEBUG_BUFFER+XN_DEBUG_PACKAGE];
static uint32_t size;
static uint32_t pos;


#define XN_DEBUG_CHANNEL 5


void xn_debug_print_ptr(char c, void * ptr) {
    if(size + 5 >= XN_DEBUG_BUFFER) {
        size = XN_DEBUG_BUFFER;
        return;
    }

    int32_t d = (int32_t)ptr;
    uint8_t *d_ptr =(uint8_t *) &d;
    data[size++] = c;
    data[size++] = d_ptr[0];
    data[size++] = d_ptr[1];
    data[size++] = d_ptr[2];
    data[size++] = d_ptr[3];
}

void xn_debug_print(char c, void * ptr)
{
    if(c == 's') {
        char * s_ptr = ptr;
        uint8_t i = 0;
        while(s_ptr[i]) {
            if(size >= XN_DEBUG_BUFFER) break;
            data[size++] = s_ptr[i++];
        }
    } else {
        xn_debug_print_ptr(c+128, ptr);
    }
}

void xn_debug_printnl()
{
    if(size >= XN_DEBUG_BUFFER) return;
    data[size++] = '\n';
}


void xn_debug_init()
{
    size  = 0;
    xn_writetxaddress(txaddress);

}

void xn_debug_send_data()
{
    while(xn_getstatus() & (1<<TX_FULL));

    xn_writepayload(&data[pos], XN_DEBUG_PACKAGE);
    pos += XN_DEBUG_PACKAGE;
}

void xn_debug_send()
{
    if(size == 0)
        return;
    pos = 0;
    if(size >= XN_DEBUG_BUFFER) {
        size = XN_DEBUG_BUFFER;
        data[XN_DEBUG_BUFFER-1]='F' + 128;
    }

    for (uint8_t i = 0; i < XN_DEBUG_PACKAGE; i++) {
        data[size + i] = 0;
    }
    xn_ceoff();
    xn_command(FLUSH_TX);
    xn_writereg(STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
    xn_writereg(CONFIG, (1<<PWR_UP)| (1<<CRCO)  | (1<<EN_CRC)); // power up, crc enabled, PTX
    xn_writereg(RF_CH, XN_DEBUG_CHANNEL);
    xn_debug_send_data();
    xn_ceon();
    while(pos < size) {
        xn_debug_send_data();
    }
    size = 0;
    while((xn_readreg(FIFO_STATUS) & 0x10) == 0);
}

