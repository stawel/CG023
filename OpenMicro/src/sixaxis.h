
#ifndef _SIXAXIS_H_
#define _SIXAXIS_H_

void sixaxis_init(void);
int sixaxis_check(void);

void sixaxis_calc(void);

void sixaxis_cal();


extern float gyro[3];
#define GYRO_FACTOR 0.0010642251536550791f  //(PI/180) / 16.4

#endif

