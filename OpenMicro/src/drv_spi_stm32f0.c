#include "project.h"
#include "drv_spi.h"
#include "binary.h"
#include "config.h"

#ifdef ENABLE_SPI_STM32F0

//TODO: check hardware PINs

inline void configure_SPI1_GPIO(void) {
    /* Enable the peripheral clock of GPIOA */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* (1) Select AF mode (10) on PA5, PA6, PA7; select output on PA4 */
    /* (2) AF0 for SPI1 signals */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 |
    GPIO_MODER_MODER6 | GPIO_MODER_MODER7))\

            | (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_1 |\
 GPIO_MODER_MODER6_1
                    | GPIO_MODER_MODER7_1); /* (1) */
    GPIOA->AFR[0] = (GPIOA->AFR[0]
            & ~( GPIO_AFRL_AFRL5 |\
 GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); /* (2) */

    spi_csoff();
}

inline void configure_SPI1(void) {
    /* Enable the peripheral clock SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Configure SPI1 in master */
    /* (1) Slave select software, master selection, BR: Fpclk/8
     CPOL and CPHA at zero (rising first edge) */
    /* (2) Slave select output disabled, 8-bit Rx fifo */
    /* (3) Enable SPI1 */
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_1; /* (1) */
    SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; /* (2) */
    SPI1->CR1 |= SPI_CR1_SPE; /* (3) */

}

void spi_init(void) {
    configure_SPI1_GPIO();
    configure_SPI1();
}

void spi_cson() {
    SPI_SS_PORT->BRR = SPI_SS_PIN;
}

void spi_csoff() {
    SPI_SS_PORT->BSRR = SPI_SS_PIN;
}

uint8_t spi_sendrecvbyte(uint8_t data) {
    *(uint8_t *) &(SPI1->DR) = data;

    while (!(SPI1->SR & SPI_SR_RXNE))
        ;

    return (uint8_t) SPI1->DR;
}

uint8_t spi_sendzerorecvbyte() {
    return spi_sendrecvbyte(0);
}

void spi_sendbyte(uint8_t data) {
    spi_sendrecvbyte(data);
}

#endif
