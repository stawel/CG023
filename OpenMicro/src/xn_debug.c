#include "project.h"
#include "xn297.h"
#include "binary.h"
#include "drv_time.h"
#include "drv_spi.h"

//#define ENABLE_DEBUG
#include "xn_debug.h"

#define XN_DEBUG_BUFFER     256
#define XN_DEBUG_PACKAGE    16
#define XN_DEBUG_CHANNEL    5
#define XN_DEBUG_TIMELIMIT  1000

static const uint8_t txaddress[5] = {0xcc, 0xcc, 0xcc, 0xcc, 0xcc};
static uint8_t buf[XN_DEBUG_BUFFER];
static uint32_t end;
static uint32_t begin;
static uint8_t rxchannel;
static uint8_t tx_active;
static uint8_t buf_full;
static uint8_t fr_setup;
static unsigned long starttime;
static unsigned long stoptime;

static int size() {
    return (XN_DEBUG_BUFFER + end - begin) % XN_DEBUG_BUFFER;
}

static int space_left() {
    if(buf_full) {
        return 0;
    }
    return XN_DEBUG_BUFFER - size();
}

static void inc_end() {
    end++;
    end %= XN_DEBUG_BUFFER;
}

static void inc_begin() {
    begin++;
    begin %= XN_DEBUG_BUFFER;
}


void xn_debug_print_ptr(char c, void * ptr, uint8_t size) {
    if(space_left() < 6) {
        buf_full = 1;
    } else {
        int32_t d = (int32_t)ptr;
        uint8_t *d_ptr =(uint8_t *) &d;

        for(uint8_t i = 0; i < size;i++) {
            buf[end] = c;
            inc_end();
            c = d_ptr[i];
        }
    }
}

void xn_debug_print_char(char c)
{
    if(space_left() < 2) {
        buf_full = 1;
    } else {
        buf[end] = c;
        inc_end();
    }
}


void xn_debug_print(char c, void * ptr)
{
    if(c == 's') {
        char * s_ptr = ptr;
        uint32_t i = 0;
        while(s_ptr[i]) {
            xn_debug_print_char(s_ptr[i++]);
        }
    } else if (c == '8') {
        xn_debug_print_ptr(c+128, ptr, 2);
    } else {
        xn_debug_print_ptr(c+128, ptr, 5);
    }
}


void xn_debug_printnl()
{
    xn_debug_print_char('\n');
}


void xn_debug_init()
{
    begin = 0;
    end  = 0;
    tx_active = 0;
    xn_writetxaddress(txaddress);
}

void xn_debug_writepayload() {
    uint8_t index = 0;
    spi_cson();
    spi_sendbyte(W_TX_PAYLOAD | W_REGISTER);
    while (index < XN_DEBUG_PACKAGE) {
        spi_sendbyte(buf[begin]);
        inc_begin();
        index++;
    }
    spi_csoff();
}

int xn_debug_send_data()
{
    if((xn_getstatus() & (1<<TX_FULL)) == 0) {
        xn_debug_writepayload();
        if(size() < XN_DEBUG_PACKAGE && buf_full) {
            buf_full = 0;
            xn_debug_print_char('F'+128);
        }
        return 1;
    }
    return 0;
}

static void xn_debug_stop() {
    xn_ceoff();
    xn_writereg(RF_SETUP, fr_setup);
    tx_active = 0;
    xn_writereg(RF_CH, rxchannel);
    xn_writereg(CONFIG, (1<<PWR_UP) | (1<<CRCO)  | (1<<EN_CRC) | (1<<PRIM_RX)); // power up, crc enabled, PTX
    xn_ceon();
    stoptime = gettime();
    LogDebug(" t:", stoptime - starttime);
}

void xn_debug_send()
{
    if(tx_active) {
        unsigned long time = gettime();
        if(time - starttime < XN_DEBUG_TIMELIMIT) {
            while((size() >= XN_DEBUG_PACKAGE) && xn_debug_send_data());
        }

        if(xn_readreg(FIFO_STATUS) & (1<<TX_EMPTY)) {
            xn_debug_stop();
        }
    }
}

static void xn_debug_start() {
    tx_active = 1;
    starttime = gettime();
    fr_setup = xn_readreg(RF_SETUP);
    LogDebug2("s:", starttime);

    if(size() >= XN_DEBUG_PACKAGE) {
        xn_ceoff();
        xn_writereg(STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
        xn_writereg(CONFIG, (1<<PWR_UP)| (1<<CRCO)  | (1<<EN_CRC)); // power up, crc enabled, PTX
        xn_writereg(RF_SETUP, (1<<RF_PWR) /*| (1<<RF_DR)*/);
        xn_writereg(RF_CH, XN_DEBUG_CHANNEL);
        xn_debug_send_data();
        xn_ceon();
    }
    xn_debug_send();
}

void xn_debug_setchannel(uint8_t channel)
{
    rxchannel = channel;
    if(!tx_active) {
        xn_debug_start();
    }
}