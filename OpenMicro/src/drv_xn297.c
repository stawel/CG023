#include <stdint.h>
#include "binary.h"
#include "drv_spi.h"

#include "project.h"
#include "xn297.h"
#include "drv_time.h"

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_tim.h"

#define ENABLE_DEBUG
#include "xn_debug.h"

#define XN297_CE_PIN    GPIO_Pin_0
#define XN297_CE_PORT   GPIOB

#define XN297_IRQ_PIN    GPIO_Pin_3
#define XN297_IRQ_PORT   GPIOA

#define XN_INIT6

#ifdef XN_INIT1 //xn927 datasheet
/* 1MBPS:
 *  p=1 ~  794us
 *  p=2 ~ 1453us
 *  p=3 ~ 2118us
 * 2MBPS:
 *  p=1 ~  687us
 *  p=2 ~ 1243us
 *  p=3 ~ 1795us
 *  p=4 ~ 2351us
 */
static const uint8_t demodcal[6] = {0x39, 0x0B, 0xDF, 0x00, 0xA7, 0x03};
static const uint8_t rfcal[8] = {0x3e, 0xCA, 0x9A, 0xB0, 0x61, 0x83, 0x2B, 0x95};
static const uint8_t bbcal[6] = {0x3f, 0x7F, 0x84, 0x67, 0x9C, 0x20};
#endif
#ifdef XN_INIT2
/* 1MBPS:
 *  p=1 ~  795us
 *  p=2 ~ 1527us
 *  p=3 ~ 2119us
 * 2MBPS:
 *  p=1 ~  687us
 *  p=2 ~ 1239us
 *  p=3 ~ 1797us
 *  p=4 ~ 2352us
 */
static const uint8_t demodcal[6] = {0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03};
static const uint8_t rfcal[8] = {0x3e, 0xc9, 0x9a, 0xb0, 0x61, 0xbb, 0xab, 0x9c};
static const uint8_t bbcal[6] = {0x3f, 0x4c, 0x84, 0x67, 0x9c, 0x20};
#endif
#ifdef XN_INIT3 //very slow transmission
/* 1MBPS:
 *  p=1 ~  919us,
 *  p=2 ~ 1710us
 *  p=3 ~ 2500us
 * 2MBPS:
 *  p=1 ~  816us
 *  p=2 ~ 1567us
 *  p=3 ~ 2181us
 *  p=4 ~
 */
static const uint8_t demodcal[6] = {0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03};
static const uint8_t rfcal[8] = {0x3e, 0xc9, 220, 0x80, 0x61, 0xbb, 0xab, 0x9c};
static const uint8_t bbcal[6] = {0x3f, 0x4c, 0x84, 0x6F, 0x9c, 0x20};
#endif
#ifdef XN_INIT4
/* 1MBPS:
 *  p=1 ~  796us
 *  p=2 ~ 1456us
 *  p=3 ~ 2118us
 * 2MBPS:
 *  p=1 ~  688us
 *  p=2 ~ 1239us
 *  p=3 ~ 1794us
 *  p=4 ~ 2355us
 */
static const uint8_t demodcal[] = { 0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03, 0xab, 0x9c };
static const uint8_t rfcal[] = { 0x3e, 0xc9, 0x9a, 0xb0, 0x61, 0xbb, 0xab, 0x9c };
static const uint8_t bbcal[] = { 0x3f, 0x4c, 0x84, 0x67, 0x9c, 0x20 };
#endif

#ifdef XN_INIT5
/* 1MBPS:
 * 2MBPS:
 *  p=1 ~  575us
 *  p=2 ~ 1016us
 *  p=3 ~ 1460us
 *  p=4 ~ 1907us
 */
static const uint8_t demodcal[6] = {0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03};
static const uint8_t rfcal[8] = {0x3e, 0xc9, 220, 0x80, 0x61, 0xbb, 0xab, 0x9c};
static const uint8_t bbcal[6] = {0x3f, 0x4c, 0x84, 0x60, 0x9c, 0x20};
#endif

#ifdef XN_INIT6
/* 1MBPS:
 *  p=1 ~  658us
 *  p=2 ~ 1184us
 *  p=3 ~ 1712us
 *  p=4 ~ 2237us
 * 2MBPS:
 *  p=1 ~  550us
 *  p=2 ~  969us
 *  p=3 ~ 1387us
 *  p=4 ~ 1811us
 */
static const uint8_t demodcal[6] = {0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03};
static const uint8_t rfcal[8] = {0x3e, 0xc9, 220, 0x80, 0x61, 0xbb, 0xab, 0x9c};
static const uint8_t bbcal[6] = {0x3f, 0x4c, 0x84, 0x00, 0x9c, 0x20};
#endif

#ifdef XN_INIT7 //xn297 datasheet (.doc)
/* 1MBPS:
 * 2MBPS:
 *  p=1 ~  924us
 *  p=2 ~ 1720us
 *  p=3 ~ 2517us
 */
static const uint8_t demodcal[6] = {0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03};
static const uint8_t rfcal[8] = {0x3e, 0xda, 0x9a, 0xB0, 0x79, 0xbb, 0xab, 0x9c};
static const uint8_t bbcal[6] = {0x3f, 0xcd, 0x3f, 0x7f, 0x9c, 0x20};
#endif

#ifdef XN_INIT8
/* 1MBPS:
 *  p=1 ~  521us
 *  p=2 ~  910us
 *  p=3 ~ 1302us
 *  p=4 ~ 1691us
 *  p=5 ~ 2086us
 *  p=6 ~
 * 2MBPS: //a lot of "PACKAGE LOST"
 *  p=1 ~    ?us
 *  p=2 ~  696us
 *  p=3 ~  980us
 *  p=4 ~ 1266us
 *  p=5 ~ 1550us
 *  p=6 ~ 1836us
 */
static const uint8_t demodcal[6] = {0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03};
static const uint8_t rfcal[8] = {0x3e, 0xda, 0x9a, 0xB0, 0x79, 0xbb, 0xab, 0x9c};
static const uint8_t bbcal[6] = {0x3f, 0xcd, 0x3f, 0x20, 0x9c, 0x20};
#endif


static volatile uint32_t irqtime;

void EXTI2_3_IRQHandler(void) {
    if ((EXTI->PR & XN297_IRQ_PIN) != 0) {
        EXTI->PR |= XN297_IRQ_PIN; // Clear the pending bit

        irqtime = gettime();
        uint8_t status = xn_writereg(STATUS,
                (1 << RX_DR) + (1 << TX_DS) + (1 << MAX_RT));

        xn_debug_irq_handler(status);
        xn_irq_handler(status);
    }
}

uint32_t xn_getirqtime() {
    return irqtime;
}

void writeregs(const uint8_t data[], uint8_t size) {

    spi_cson();
    for (uint8_t i = 0; i < size; i++) {
        spi_sendbyte(data[i]);
    }
    spi_csoff();
    delay(1000);
}

static void configure_ce_GPIO() {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = XN297_CE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(XN297_CE_PORT, &GPIO_InitStructure);
}

static void configure_IRQ_GPIO() {
    EXTI->IMR = XN297_IRQ_PIN;
    EXTI->FTSR = XN297_IRQ_PIN;

    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 0);
}

void xn_ceon() {
    XN297_CE_PORT->BSRR = XN297_CE_PIN;
}

void xn_ceoff() {
    XN297_CE_PORT->BRR = XN297_CE_PIN;
}

void xn_init() {
    configure_ce_GPIO();
    configure_IRQ_GPIO();
    xn_ceon();

    writeregs(bbcal, sizeof(bbcal));
    writeregs(rfcal, sizeof(rfcal));
    writeregs(demodcal, sizeof(demodcal));

    xn_debug_init();
}

uint8_t xn_writereg(uint8_t reg, uint8_t val) {
    reg = reg & 0x1F;
    reg = reg | 0x20;
    spi_cson();
    uint8_t status = spi_sendrecvbyte(reg);
    spi_sendbyte(val);
    spi_csoff();
    return status;
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

void xn_writedata(uint8_t reg, const uint8_t *addr, uint8_t size) {
    uint8_t index = 0;
    spi_cson();
    spi_sendbyte(reg | W_REGISTER);
    while (index < size) {
        spi_sendbyte(addr[index]);
        index++;
    }
    spi_csoff();
}

void xn_writerxaddress(const uint8_t *addr) {
    xn_writedata(RX_ADDR_P0, addr, 5);
}

void xn_writetxaddress(const uint8_t *addr) {
    xn_writedata(TX_ADDR, addr, 5);
}

void xn_writepayload(const uint8_t data[], uint8_t size) {
    xn_writedata(W_TX_PAYLOAD, data, size);
}

//#define DEBUG_XN___
void xn_setchannel(uint8_t channel) {
    xn_debug_setchannel(channel);
#ifdef DEBUG_XN___
    xn_ceoff();
    xn_writereg(RF_CH, channel);
    xn_writereg(CONFIG, (1<<PWR_UP) | (1<<CRCO) | (1<<EN_CRC) | (1<<PRIM_RX)); // power up, crc enabled, PTX
    xn_ceon();
#endif
}

uint8_t xn_getstatus() {
    return xn_command(0);
}
