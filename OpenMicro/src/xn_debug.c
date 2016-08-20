#include "project.h"
#include "xn297.h"
#include "binary.h"
#include "drv_time.h"
#include "drv_spi.h"

#define ENABLE_DEBUG
#include "xn_debug.h"

#define XN_DEBUG_BUFFER     256
#define XN_DEBUG_PACKAGE        15
#define XN_DEBUG_PAYLOAD_SIZE   (XN_DEBUG_PACKAGE+1)   //+1 - package_nr
#define XN_DEBUG_CHANNEL    5
//TODO: fix timelimit
#define XN_DEBUG_TIMELIMIT  500

static const uint8_t txaddress[5] = { 0xcc, 0xcc, 0xcc, 0xcc, 0xcc };
static uint8_t buf[XN_DEBUG_BUFFER];
static uint8_t rxchannel;
static uint8_t tx_active;
static uint8_t fr_setup;
static unsigned long starttime;
static unsigned long stoptime;
static uint8_t packages_send;

static volatile uint32_t i_end;
static volatile uint32_t i_begin;
static volatile uint8_t i_buf_full;

static int buf_size() {
    return (XN_DEBUG_BUFFER + i_end - i_begin) % XN_DEBUG_BUFFER;
}

static int ensure_free_space(uint8_t space) {
    if (i_buf_full)
        return 0;

    int retu = 1;
    __disable_irq();
    if (XN_DEBUG_BUFFER - buf_size() < space) {
        retu = 0;
        i_buf_full = 1;
    }
    __enable_irq();
    return retu;
}

static void inc_end() {
    i_end++;
    i_end %= XN_DEBUG_BUFFER;
}

static void inc_begin() {
    i_begin++;
    i_begin %= XN_DEBUG_BUFFER;
}

void xn_debug_print_data(char c, uint32_t d, uint8_t size) {
    if (ensure_free_space(6)) {
        uint8_t *d_ptr = (uint8_t *) &d;
        c += 128;
        for (uint8_t i = 0; i < size; i++) {
            buf[i_end] = c;
            inc_end();
            c = d_ptr[i];
        }
    }
}

void xn_debug_print_char(char c) {
    if (ensure_free_space(2)) {
        buf[i_end] = c;
        inc_end();
    }
}

void xn_debug_print_string(char * ptr) {
    uint32_t i = 0;
    while (ptr[i]) {
        xn_debug_print_char(ptr[i++]);
    }
}
void xn_debug_print_long(long x) {
    xn_debug_print_data('i', x, 5);
}
void xn_debug_print_u8(uint8_t x) {
    xn_debug_print_data('8', x, 2);
}

void xn_debug_print_float(float x) {
    uint32_t *ptr = (uint32_t *) &x;
    xn_debug_print_data('f', *ptr, 5);
}

void xn_debug_printnl() {
    xn_debug_print_char('\n');
}

void xn_debug_init() {
    i_begin = 0;
    i_end = 0;
    tx_active = 0;
    xn_writetxaddress(txaddress);
}

void xn_debug_writepayload() {
    static uint8_t package_nr = 0;
    uint8_t index = 0;
    spi_cson();
    spi_sendbyte(W_TX_PAYLOAD | W_REGISTER);
    spi_sendbyte(package_nr++);
    while (index < XN_DEBUG_PACKAGE) {
        spi_sendbyte(buf[i_begin]);
        inc_begin();
        index++;
    }
    spi_csoff();
    packages_send++;
}

int xn_debug_send_data() {
    if ((xn_getstatus() & (1 << TX_FULL)) == 0) {
        xn_debug_writepayload();
        if (buf_size() < XN_DEBUG_PACKAGE && i_buf_full) {
            i_buf_full = 0;
            xn_debug_print_char('F' + 128);
        }
        return 1;
    }
    return 0;
}

static void xn_debug_stop_transmission() {
    xn_ceoff();
    xn_writereg(RF_SETUP, fr_setup);
    tx_active = 0;
    xn_writereg(RF_CH, rxchannel);
    xn_writereg(CONFIG,
            (1 << PWR_UP) | (1 << CRCO) | (1 << EN_CRC) | (1 << PRIM_RX)); // power up, crc enabled, PTX
    xn_ceon();
    stoptime = gettime();

    LogDebug("xn_debug s: ", starttime, " p: ", packages_send, " t: ", stoptime - starttime);
}

void xn_debug_send_or_stop() {
    if (tx_active) {
        unsigned long time = gettime();
        if (time - starttime < XN_DEBUG_TIMELIMIT
                && buf_size() >= XN_DEBUG_PACKAGE) {
            xn_debug_send_data();
        } else if (xn_readreg(FIFO_STATUS) & (1 << TX_EMPTY)) {
            xn_debug_stop_transmission();
        }
    }
}

static void xn_debug_start_transmission() {
    tx_active = 1;
    packages_send = 0;
    starttime = gettime();
    fr_setup = xn_readreg(RF_SETUP);

    if (buf_size() >= XN_DEBUG_PACKAGE) {
        xn_ceoff();
        xn_writereg(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
        xn_writereg(CONFIG, (1 << PWR_UP) | (1 << CRCO) | (1 << EN_CRC)); // power up, crc enabled, PTX
        xn_writereg(RF_SETUP, (1 << RF_PWR) /*| (1<<RF_DR)*/);
        xn_writereg(RF_CH, XN_DEBUG_CHANNEL);
        xn_debug_send_data();
        xn_ceon();
    }
    xn_debug_send_or_stop();
}

void xn_debug_setchannel(uint8_t channel) {
    rxchannel = channel;
    if (!tx_active) {
        xn_debug_start_transmission();
    }
}

void xn_debug_irq_handler(uint8_t status) {
    if (status & (1 << TX_DS)) {
        xn_debug_send_or_stop();
    }
}
