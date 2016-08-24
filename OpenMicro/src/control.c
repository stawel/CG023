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
#include <math.h>

#include "pid.h"
#include "config.h"
#include "util.h"
#include "drv_pwm.h"
#include "control.h"
#include "defines.h"
#include "drv_time.h"
#include "sixaxis.h"
#include "drv_fmc.h"
#include "sixaxis.h"

#define ENABLE_DEBUG
#include "xn_debug.h"

extern float rx[7];
extern int failsafe;
extern float pidoutput[PIDNUMBER];

extern float angleerror[3];
extern float attitude[3];

int onground = 1;

float error[PIDNUMBER];
float motormap(float input);

float yawangle;

extern float looptime;

extern char auxchange[AUXNUMBER];
extern char aux[AUXNUMBER];

extern float apid(int x);

#ifdef NOMOTORS
// to maintain timing or it will be optimized away
float tempx[4];
#endif

unsigned long timecommand = 0;

extern int controls_override;
extern float rx_override[];
extern int acro_override;

void check_onground() {
    if (onground) {
        if (rx[1] < -0.8) {
            if (!timecommand)
                timecommand = gettime();
            if (gettime() - timecommand > 3e6) {
                // do command

                sixaxis_cal();  // for flashing lights
                extern float accelcal[3];
#ifndef ACRO_ONLY
                fmc_write(accelcal[0] + 127, accelcal[1] + 127);
#endif
                // reset loop time so max loop time is not exceeding
                extern unsigned lastlooptime;
                lastlooptime = gettime();
                timecommand = 0;
            }
        } else
            timecommand = 0;
    }

}

void set_motors(float throttle) {
    float mix[4];

    mix[MOTOR_FL] = throttle + pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW];       // FL
    mix[MOTOR_FR] = throttle - pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW];       // FR
    mix[MOTOR_BR] = throttle - pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW];       // BR
    mix[MOTOR_BL] = throttle + pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW];       // BL

    for (int i = 0; i <= 3; i++) {
#ifdef MOTOR_FILTER
        mix[i] = motorfilter(mix[i], i);
#endif
#ifdef CLIP_FF
        mix[i] = clip_ff(mix[i], i);
#endif
#ifndef NOMOTORS
#ifndef MOTORS_TO_THROTTLE
        pwm_set(i, mix[i]);
#else
        pwm_set(i, throttle);
#endif
#else
        tempx[i] = mix[i];
#endif
        if (mix[i] < 0)
            mix[i] = 0;
        if (mix[i] > 1)
            mix[i] = 1;
    }
}

void disable_motors() {
    // motors off
    for (int i = 0; i <= 3; i++) {
        pwm_set(i, 0);
#ifdef MOTOR_FILTER
        // reset the motor filter
        motorfilter(0, i);
#endif
    }

}

void control(void) {

    // check for accelerometer calibration command
    check_onground();

// rate only build
    //TODO: change ROLL, PITCH, YAW
/*    error[ROLL] = rx[ROLL] * (float) MAX_RATE * DEGTORAD + gyro[1]*0.1;
    error[PITCH] = rx[PITCH] * (float) MAX_RATE * DEGTORAD - gyro[0]*0.1;
    error[YAW] = rx[YAW] * (float) MAX_RATEYAW * DEGTORAD - gyro[YAW]*0.1;
*/

    error[ROLL] = rx[ROLL] * (float) MAX_RATE * DEGTORAD + world_z_quaternion[1];
    error[PITCH] = rx[PITCH] * (float) MAX_RATE * DEGTORAD + world_z_quaternion[2];
    error[YAW] = rx[YAW] * (float) MAX_RATEYAW * DEGTORAD - gyro[YAW];


/*
    error[ROLL] = world_z_quaternion[1];
    error[PITCH] = world_z_quaternion[2];
    error[YAW] = - gyro[YAW];
*/
//    error[YAW] = 0.;
//    error[ROLL] = 0.;
//    error[PITCH] = 0.;

//    LogDebug("R: ", rx[ROLL] * (float) MAX_RATE * DEGTORAD, " ", rx[PITCH] * (float) MAX_RATE * DEGTORAD, " ", rx[YAW] * (float) MAX_RATE * DEGTORAD);
//    LogDebug("E: ", error[ROLL], " ", error[PITCH], " ", error[YAW]);

    pid_precalc();

    pid(ROLL);
    pid(PITCH);
    pid(YAW);

    extern float ierror[3];
    LogDebug("r: ", error[ROLL], " ", pidoutput[ROLL], " ", ierror[ROLL]);

//    LogDebug("O: ", pidoutput[ROLL], " ", pidoutput[PITCH], " ", pidoutput[YAW]);


    float throttle;

// map throttle so under 10% it is zero	
    if (rx[3] < 0.1f)
        throttle = 0;
    else
        throttle = (rx[3] - 0.1f) * 1.11111111f;

// turn motors off if throttle is off and pitch / roll sticks are centered
    if (failsafe || throttle < 0.001f) {
        disable_motors();
        onground = 1;

    } else {
        onground = 0;
        set_motors(throttle);

    }		// end motors on

}

//************************************************************************************

float hann_lastsample[4];
float hann_lastsample2[4];

// hanning 3 sample filter
float motorfilter(float motorin, int number) {
    float ans = motorin * 0.25f + hann_lastsample[number] * 0.5f
            + hann_lastsample2[number] * 0.25f;

    hann_lastsample2[number] = hann_lastsample[number];
    hann_lastsample[number] = motorin;

    return ans;
}

float clip_feedforward[4];
// clip feedforward adds the amount of thrust exceeding 1.0 ( max) 
// to the next iteration(s) of the loop
// so samples 0.5 , 1.5 , 0.4 would transform into 0.5 , 1.0 , 0.9;

float clip_ff(float motorin, int number) {

    if (motorin > 1.0f) {
        clip_feedforward[number] += (motorin - 1.0f);
        //cap feedforward to prevent windup
        if (clip_feedforward[number] > .5f)
            clip_feedforward[number] = .5f;
    } else if (clip_feedforward[number] > 0) {
        float difference = 1.0f - motorin;
        motorin = motorin + clip_feedforward[number];
        if (motorin > 1.0f) {
            clip_feedforward[number] -= difference;
            if (clip_feedforward[number] < 0)
                clip_feedforward[number] = 0;
        } else
            clip_feedforward[number] = 0;

    }
    return motorin;
}

