#ifndef PROTOCOL_TIMER_H
#define PROTOCOL_TIMER_H

#include <stdint.h>

void protocol_timer_init(uint16_t period_ms);
void protocol_timer_reset();

//user defined
void protocol_timer_irq();

#endif //PROTOCOL_TIMER_H
