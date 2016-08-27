#!/usr/bin/python
import sys
import serial
import struct
import collections
import time

port_name = '/dev/ttyUSB0'
uart = serial.Serial(port_name,2000000,timeout=0.010)
crc = 2         #bytes
package_len = 15 + 2 + crc #c('P')+package_nr+CRC


raw_stream = collections.deque(maxlen=2000)
raw_buffer = collections.deque(maxlen=2000)
line_buffer = collections.deque(maxlen=200)
last_line = [];

package_nr = 0;

def parse_float(x):
    temp = struct.pack("i", int(x))

def c(char):
    return chr(ord(char)+128)

def unpack4bytes(c, t):
    if len(raw_buffer) < 4:
        raw_buffer.appendleft(c)
        raise NameError("My error char: "+ t);
    f = ''.join([ raw_buffer.popleft() for x in range(0,4) ])
    return struct.unpack(t, f)[0]

def unpack1bytes(c, t):
    if len(raw_buffer) < 1:
        raw_buffer.appendleft(c)
        raise NameError("My error char: "+ t);
    f = ''.join([ raw_buffer.popleft() for x in range(0,1) ])
    return struct.unpack(t, f)[0]


def endline(t = []):
    global last_line
    for i in t:
        add(str(i))
    add('\n')
    line = ''.join(last_line)
    line_buffer.append(line)
    last_line = []

def add(t):
    sys.stdout.write(str(t))
    last_line.append(str(t))



#  CRC TODO: rewrite ##########################################

bit_reverse = [
  0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0, 
  0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
  0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
  0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
  0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
  0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
  0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
  0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
  0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
  0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
  0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
  0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
  0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
  0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
  0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
  0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
  0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
  0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
  0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
  0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
  0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
  0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
  0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
  0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
  0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
  0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
  0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
  0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
  0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
  0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
  0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
  0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff]


xn297_scramble = [
  0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
  0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
  0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
  0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
  0x8e, 0xc5, 0x2f]


def crc16_update(crc, a):
    polynomial = 0x1021
    crc = crc ^(a << 8)
    for i in range(0,8):
        if crc & 0x8000 > 0:
            crc = (crc << 1) ^ polynomial
        else:
            crc = crc << 1

    crc = crc & 0xffff
    return crc

xn297_crc_xorout = [
    0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C, # 1st entry is missing, probably never needed
    0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828,
    #info: on M9912 this number differs
    0x8148 ^ 0x03,
    # it's used for 3-byte address w/ 0 byte payload only
    0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
    0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401, 
    0x2138, 0x129F, 0xB3A0, 0x2988]


ccc = 0;

address = [0xcc, 0xcc, 0xcc, 0xcc, 0xcc]


def check_crc(b0, l, crc2):
    global ccc
    crc = 0xb5d2;
    p =0
    for i in address:
        crc = crc16_update(crc,i ^ xn297_scramble[5-p-1])
        p= p+1

    crc = crc16_update(crc,bit_reverse[b0 ^ xn297_scramble[p]])
    p= p+1

    for i in l:
        crc = crc16_update(crc,bit_reverse[ord(i) ^ xn297_scramble[p]])
        p=p+1

    #TODO: ??
    crc2= crc2^0x9360
#    print "CRC:", ccc , hex(crc), " ", hex()
    return crc2 == crc

### END CRC #######################################

def parse_raw_buffer():
    try:
        while len(raw_buffer) > 0:
            char = raw_buffer.popleft()
#                add('[' + str(ord(char)) + ']')
            if char == '\n':
                endline()
            elif char == c('F'):
                endline()
                endline(['[COPTER BUFFER FULL]'])
            elif char == c('f'):
                add(unpack4bytes(char, 'f'))
            elif char == c('i'):
                add(unpack4bytes(char, 'i'))
            elif char == c('8'):
                add(unpack1bytes(char, 'B'))
            else:
                add(char)
    except NameError as e:
        pass

def parse_raw_stream():
    global package_nr, last_line
    char = raw_stream.popleft()
    if char == c('P'):
        current_package_nr = ord(raw_stream.popleft())
        if (current_package_nr - package_nr + 256)% 256 != 1:
            endline()
            endline(['[PACKAGE LOST, last: ', str(package_nr),' new: ', str(current_package_nr), ' ]'])
        package_nr = current_package_nr
        l = [raw_stream.popleft() for i in range(0, package_len - 2 - crc)]
#        last_line = last_line + ['{']+l+['}']
        l_ok = True
        if crc == 2:
            crc_b1 = raw_stream.popleft()
            crc_b2 = raw_stream.popleft()
            crc2 = (ord(crc_b1)<<8) + ord(crc_b2)
            l_ok = check_crc(current_package_nr, l, crc2)

        if l_ok:
            raw_buffer.extend(l)
        else:
            endline(['[PACKAGE CRC ERROR ', hex(crc2), ']'])
    else:
        endline()
        endline(['[ERROR: \'P\' not found]'])
        for i in range(0, package_len - 2):
            char = raw_stream.popleft()
            if char == c('P'):
                raw_stream.appendleft(char)
            else:
                raw_buffer.append(char)



def uart_parse_stream():
    r = uart.read(uart.inWaiting())
    raw_stream.extend(r);
    while len(raw_stream) >= package_len:
        parse_raw_stream()
        parse_raw_buffer()

#    endline(["raw_stream len: ", str(len(raw_stream)), "raw_buffer len: ", str(len(raw_buffer)), " inWating: ", str(uart.inWaiting())])
    sys.stdout.flush()


if __name__ == "__main__":
    while True:
        uart_parse_stream()
        time.sleep(0.05)
