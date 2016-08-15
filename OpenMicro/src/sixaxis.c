/*
 The MIT License (MIT)

 Copyright (c) 2016 silverx

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#include <inttypes.h>
#include "binary.h"
#include "sixaxis.h"
#include "drv_time.h"

#include "util.h"
#include "config.h"
#include "led.h"

#include "drv_serial.h"
#include "drv_softi2c.h"

#include <math.h>
#include "3dmath.h"

#define ENABLE_DEBUG
#include "xn_debug.h"

#include "debug.h"
extern debug_type debug;

void sixaxis_init(void) {
    // gyro soft reset
    softi2c_write( SOFTI2C_GYRO_ADDRESS, 107, 128);
    delay(40000);

    // set pll to 1, clear sleep bit old type gyro (mpu-6050)
    softi2c_write( SOFTI2C_GYRO_ADDRESS, 107, 1);
    int newboard = !(0x68 == softi2c_read( SOFTI2C_GYRO_ADDRESS, 117));

    softi2c_write( SOFTI2C_GYRO_ADDRESS, 28, B00011000);	// 16G scale

    // acc lpf for the new gyro type
    //       0-6 ( same as gyro)
    if (newboard) {
        softi2c_write( SOFTI2C_GYRO_ADDRESS, 29, ACC_LOW_PASS_FILTER);
    }

    // gyro scale 2000 deg (FS =3)
    softi2c_write( SOFTI2C_GYRO_ADDRESS, 27, 24);

    // Gyro DLPF low pass filter
    softi2c_write( SOFTI2C_GYRO_ADDRESS, 26, GYRO_LOW_PASS_FILTER);
}

int sixaxis_check(void) {
    // read "who am I" register

    int id = softi2c_read( SOFTI2C_GYRO_ADDRESS, 117);
    // new board returns 78h (unknown gyro maybe mpu-6500 compatible) marked m681
    // old board returns 68h (mpu - 6050)
    // a new (rare) gyro marked m540 returns 7Dh
#ifdef DEBUG
    debug.gyroid = id;
#endif
#ifdef DISABLE_GYRO_CHECK
    return 1;
#endif
    return (0x78 == id || 0x68 == id || 0x7d == id);
}

float accel[3];
float gyro[3];

float accelcal[3];
float gyrocal[3];

//TODO: do a #define
float sensor_rotation[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

float lpffilter(float in, int num);

void sixaxis_read(void) {
    uint8_t data[16];
    softi2c_readdata( SOFTI2C_GYRO_ADDRESS, 59, data, 14);

    v3d_set(accel, &data[0]);
    v3d_rotate(accel, sensor_rotation);

    v3d_set(gyro, &data[8]);
    //TODO: *-1.0f ??
    v3d_mulf(gyro, 1.0f);
    v3d_rotate(gyro, sensor_rotation);

    v3d_add(gyro, gyrocal);
    v3d_mulf(gyro, 0.061035156f * 0.017453292f);

    //LogDebug("6ax: ", gyro[0], " ", gyro[1], " ", gyro[2], "\t", accel[0], " ", accel[1], " ", accel[2]);
}

#define CAL_TIME 2000*1000
//TODO: set errors
#define ACCELCAL_ERROR  10.0f
#define GYROCAL_ERROR   10.0f

void sixaxis_cal() {
    unsigned long start_time = gettime();
    unsigned long time;
    unsigned long count = 0;

    float new_gyrocal[3];
    float new_accelcal[3];

    float sum_gyrocal[3];
    float sum_accelcal[3];

    v3d_zero(sum_gyrocal);
    v3d_zero(sum_gyrocal);

    do {
        time = gettime();
        v3d_copy(new_gyrocal, gyro);
        v3d_copy(new_accelcal, accel);

        sixaxis_read();

        v3d_sub(new_gyrocal, gyro);
        v3d_sub(new_accelcal, accel);

        if (v3d_magnitude(new_gyrocal) > GYROCAL_ERROR
                || v3d_magnitude(new_accelcal) > ACCELCAL_ERROR) {
            LogDebug("Calibration interrupted!");
            return;
        }

        v3d_add(sum_gyrocal, gyro);
        v3d_add(sum_accelcal, accel);

        count++;
        if (((count) / 500) & 1) {
            ledon(B00000101);
            ledoff(B00001010);
        } else {
            ledon(B00001010);
            ledoff(B00000101);
        }

        delay(1000);

    } while (time - start_time < CAL_TIME);

    float m = 1.0f/count;
    v3d_mulf(sum_gyrocal, m);
    v3d_mulf(sum_accelcal, m);

    v3d_copy(accelcal, sum_accelcal);
    v3d_sub(gyrocal, sum_gyrocal);
    LogDebug("6cb: ", count , " ", gyrocal[0], " ", gyrocal[1], " ", gyrocal[2], "\t",
            accelcal[0], " ", accelcal[1], " ", accelcal[2]);
}

