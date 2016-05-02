#include "project.h"
#include "xn297.h"
#include "binary.h"
#include "drv_time.h"

#define ENABLE_DEBUG
#include "xn_debug.h"

#define XN_DEBUG_BUFFER 256
#define XN_DEBUG_PACKAGE 16

static const uint8_t txaddress[5] = {0xcc, 0xcc, 0xcc, 0xcc, 0xcc};
static uint8_t data[XN_DEBUG_BUFFER];
static uint32_t size;
static uint32_t pos;


#define XN_DEBUG_CHANNEL 5


void xn_debug_print_ptr(char c, void * ptr) {
    if(size + 5 >= XN_DEBUG_BUFFER) {
        for(;size<XN_DEBUG_BUFFER;size++) {
            data[size] = 'F'+128;
        }
    } else {
        int32_t d = (int32_t)ptr;
        uint8_t *d_ptr =(uint8_t *) &d;
        data[size++] = c;
        data[size++] = d_ptr[0];
        data[size++] = d_ptr[1];
        data[size++] = d_ptr[2];
        data[size++] = d_ptr[3];
    }
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
    if(size < XN_DEBUG_BUFFER) {
        data[size++] = '\n';
    }
}


void xn_debug_init()
{
    size  = 0;
    xn_writetxaddress(txaddress);

}

void xn_debug_send_data()
{
    for (uint32_t i = size; i < pos + XN_DEBUG_PACKAGE; i++) {
        data[i] = 0;
    }

    while(xn_getstatus() & (1<<TX_FULL));

    xn_writepayload(&data[pos], XN_DEBUG_PACKAGE);
    pos += XN_DEBUG_PACKAGE;
}

void xn_debug_send()
{
    unsigned long t0 = gettime();
    LogDebug2("t:");
    if(size == 0)
        return;
    pos = 0;
    if(size == XN_DEBUG_BUFFER) {
        data[XN_DEBUG_BUFFER-1]='F' + 128;
    }

    unsigned long t1 = gettime();
    xn_ceoff();
    uint8_t fr_setup = xn_readreg(RF_SETUP);
    xn_writereg(STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
    xn_writereg(CONFIG, (1<<PWR_UP)| (1<<CRCO)  | (1<<EN_CRC)); // power up, crc enabled, PTX
    xn_writereg(RF_SETUP, (1<<RF_PWR) /*| (1<<RF_DR)*/);
    xn_writereg(RF_CH, XN_DEBUG_CHANNEL);
    xn_debug_send_data();
    xn_ceon();
    unsigned long t2 = gettime();

    while(pos < size) {
        xn_debug_send_data();
    }
    size = 0;
    unsigned long t3 = gettime();

    while((xn_readreg(FIFO_STATUS) & 0x10) == 0);
    xn_ceoff();
    xn_writereg(RF_SETUP, fr_setup);

    unsigned long t4 = gettime();
    LogDebug("duzo tekstu i jeszcze pawel ", t4-t0 , " ", t4-t3);
}

