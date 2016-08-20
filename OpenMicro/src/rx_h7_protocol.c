/*
 The MIT License (MIT)

 Copyright (c) 2016 stawel

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

#include "binary.h"
#include "drv_spi.h"

#include "project.h"
#include "xn297.h"
#include "drv_time.h"
#include "config.h"
#include "defines.h"

#include "rx_bayang.h"
#include "util.h"

#include "protocol_timer.h"

#define ENABLE_DEBUG
#include "xn_debug.h"

#ifdef RX_H7_PROTOCOL

extern float rx[4];

extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];

#define H7_FLIP_MASK  0x80 // right shoulder (3D flip switch), resets after aileron or elevator has moved and came back to neutral
#define H7_F_S_MASK  0x01
#define H7_FLAG_VIDEO  0x10

#define PACKET_SIZE 9   // packets have 9-byte payload
#define PACKET_PERIOD    2625

#define ALWAYS_CHANGE_CHANNEL

int failsafe = 0;
uint8_t rxdata[PACKET_SIZE];
int rxmode = 0;

static uint8_t rxaddress[5] = { 0xcc, 0xcc, 0xcc, 0xcc, 0xcc };
//{0xb2, 0xbe, 0x00, 0xcc, 0xcc};

static const uint8_t H7_freq[] = { 0x02, 0x48, 0x0C, 0x3e, 0x16, 0x34, 0x20,
        0x2A, 0x2A, 0x20, 0x34, 0x16, 0x3e, 0x0c, 0x48, 0x02 };

static int channeloffset;
static uint8_t channel_idx;
static unsigned long lastrxtime;
static unsigned long lastrxchannel;

void rx_init() {

    xn_init();
    xn_writerxaddress(rxaddress);

    xn_writereg(EN_AA, 0);	// aa disabled
    xn_writereg(EN_RXADDR, 1); // pipe 0 only
    xn_writereg(RF_SETUP, B00000001); // lna high current on ( better performance )
    xn_writereg(RX_PW_P0, PACKET_SIZE); // payload size
    xn_writereg(SETUP_RETR, 0); // no retransmissions ( redundant?)
    xn_writereg(SETUP_AW, 3); // address size (5 bytes)
    xn_command(FLUSH_RX);
    xn_setchannel(22);  // bind  channel
    protocol_timer_init(PACKET_PERIOD);
}

static uint8_t checksum_offset = 0;

static uint8_t calc_checksum(void) {
    uint8_t result = checksum_offset;
    for (uint8_t i = 0; i < 8; i++)
        result += rxdata[i];
    return result & 0xFF;
}

static uint8_t check_checksum() {
    return rxdata[8] == calc_checksum();
}

static void set_channel(uint8_t ch) {
    channel_idx = ch & 0xf;
    xn_setchannel(H7_freq[channel_idx] + channeloffset); // Set channel frequency
}

void decode_h7(void) {

    unsigned long timediff = gettime() - lastrxtime;
    if (timediff > FAILSAFETIME) {  //  failsafe
        failsafe = 1;
        rx[0] = 0;
        rx[1] = 0;
        rx[2] = 0;
        rx[3] = 0;
    } else {

        failsafe = 0;
        rx[3] = 0.00390625f * (225 - rxdata[0]);
        rx[1] = (((int) rxdata[3]) - 112) * 0.00166666f;
        rx[0] = (((int) rxdata[2]) - 112) * 0.00166666f; // roll
        rx[2] = (-((int) rxdata[1]) + 112) * 0.00166666f;

        //rxdata[4] L-R: default:32, (63..1)
        //rxdata[5] F-B: default:32, (1..63)
        //rxdata[6] default:0, 1: F/S, 128: flip

        aux[0] = (rxdata[6] & H7_FLIP_MASK) ? 1 : 0;
        aux[1] = (rxdata[6] & H7_FLAG_VIDEO) ? 1 : 0;
        aux[2] = (rxdata[6] & H7_F_S_MASK) ? 1 : 0; //??

#ifndef DISABLE_EXPO
        rx[0] = rcexpo ( rx[0] , EXPO_XY );
        rx[1] = rcexpo ( rx[1] , EXPO_XY );
        rx[2] = rcexpo ( rx[2] , EXPO_YAW );
#endif

        for (int i = 0; i < AUXNUMBER - 2; i++) {
            auxchange[i] = 0;
            if (lastaux[i] != aux[i])
                auxchange[i] = 1;
            lastaux[i] = aux[i];
        }
    }
}

void checkrx(void) {
    if (rxmode != RXMODE_BIND && check_checksum()) {
        __disable_irq();
        decode_h7();
        __enable_irq();
    }
}

static uint8_t bindrx() {
    if (rxdata[0] == 0x20) {    // bind packet received
        rxaddress[0] = rxdata[4];
        rxaddress[1] = rxdata[5];
        rxaddress[2] = 0;
        rxmode = RXMODE_NORMAL;
        xn_writerxaddress(rxaddress);

        channeloffset = (((rxdata[7] & 0xf0) >> 4) + (rxdata[7] & 0x0f)) % 8;
        xn_command(FLUSH_RX);
        checksum_offset = rxdata[7];
        return 1;
    }
    return 0;
}

uint8_t internal_rx() {
    xn_readpayload(rxdata, PACKET_SIZE);
    xn_writereg(STATUS, (1 << RX_DR));
    if (rxmode == RXMODE_BIND) {	// rx startup , bind mode
        return bindrx();
    } else {	// normal mode
        return check_checksum();
    }
}

void change_channel() {
    unsigned long time = gettime();
    unsigned long timediff = time - lastrxtime;
    unsigned long package = timediff / PACKET_PERIOD;

    if (package > 16) {
        package /= 16;
    }
    package++;
#ifndef ALWAYS_CHANGE_CHANNEL
    if(((package + lastrxchannel) & 0xf) != channel_idx) {
        set_channel(package + lastrxchannel);
    }
#else
    set_channel(package + lastrxchannel);
#endif

    LogDebug("rx p: ", package, " i: ", channel_idx, " c: ",
            (int)(H7_freq[channel_idx] + channeloffset), " td: ", timediff);
}

void protocol_timer_irq_handler() {
    change_channel();
}

void xn_irq_handler(uint8_t status) {
    uint8_t current_channel_idx = channel_idx;
    if (status & (1 << RX_DR)) {
        if (internal_rx()) {
            protocol_timer_reset();
            lastrxtime = xn_getirqtime();
            lastrxchannel = current_channel_idx;
            failsafe = 0;
            change_channel();
        }
    }
}

// end H7 proto
#endif

