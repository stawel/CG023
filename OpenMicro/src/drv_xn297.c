#include "binary.h"
#include "drv_spi.h"

#include "project.h"
#include "xn297.h"
#include "xn_debug.h"
#include "drv_time.h"
#include "stm32f0xx.h"

#define XN297_CE_PIN    GPIO_Pin_0
#define XN297_CE_PORT   GPIOB

#define XN297_IRQ_PIN    GPIO_Pin_3
#define XN297_IRQ_PORT   GPIOA


static const uint8_t bbcal[6] = { 0x3f, 0x4c, 0x84, 0x6F, 0x9c, 0x20 };
static const uint8_t rfcal[8] = {0x3e, 0xc9, 220, 0x80, 0x61, 0xbb, 0xab, 0x9c };
static const uint8_t demodcal[6] = { 0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03 };

static volatile uint32_t irqtime;


void EXTI2_3_IRQHandler(void)
{
    if ((EXTI->PR & (1<<3)) != 0) {
        EXTI->PR |= (1<<3); /* Clear the pending bit */
        irqtime = gettime();
    }
}

uint32_t xn_getirqtime()
{
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
    //SYSCFG->EXTICR[3] = SYSCFG_EXTICR1_EXTI3_PA;
    EXTI->IMR = 1<<3;
    EXTI->FTSR = 1<<3;

    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn,0);
}


void xn_ceon() {
    XN297_CE_PORT->BSRR = XN297_CE_PIN;
}

void xn_ceoff() {
    XN297_CE_PORT->BRR = XN297_CE_PIN;
}


void xn_init()
{
    configure_ce_GPIO();
    configure_IRQ_GPIO();
    xn_ceon();

    //writeregs(bbcal, sizeof(bbcal));
    writeregs(rfcal, sizeof(rfcal));
    writeregs(demodcal, sizeof(demodcal));

    xn_debug_init();
}

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

void xn_writedata(uint8_t reg,const uint8_t *addr, uint8_t size) {
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

void xn_setchannel(uint8_t channel) {
    xn_debug_send();
    xn_ceoff();
    xn_writereg(RF_CH, channel);
    xn_writereg(CONFIG, (1<<PWR_UP) | (1<<CRCO)  | (1<<EN_CRC) | (1<<PRIM_RX)); // power up, crc enabled, PTX
    xn_ceon();
}

uint8_t xn_getstatus()
{
    return xn_command(0);
}
