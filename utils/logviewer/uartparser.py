#!/usr/bin/python
import sys
import serial
import struct
import collections


port_name = '/dev/ttyUSB0'
uart = serial.Serial(port_name,2000000,timeout=0.010)
package_len = 15 + 2 #c('P')+package_nr


raw_stream = collections.deque(maxlen=2000)
raw_buffer = collections.deque(maxlen=2000)
line_buffer = collections.deque(maxlen=200)
last_line = [];

package_nr = 0;

def parse_float(x):
    temp = struct.pack("i", int(x))

def c(char):
    return chr(ord(char)+128)

def unpack4bytes(t):
    f = ''.join([ raw_buffer.popleft() for x in range(0,4) ])
    return struct.unpack(t, f)[0]

def endline(t = []):
    global last_line
    last_line = last_line + t
    line = ''.join(last_line)
#    print line
    line_buffer.append(line)
    last_line = []

def add(t):
    last_line.append(str(t))

def parse_raw_buffer():
    char = raw_buffer.popleft()
#    add('[' + str(ord(char)) + ']')
    if char == '\n':
        endline()
    elif char == c('F'):
        endline()
        endline(['[COPTER BUFFER FULL]'])
    elif char == c('f'):
        add(unpack4bytes('f'))
    elif char == c('i'):
        add(unpack4bytes('i'))
    else:
        add(char)

def parse_raw_stream():
    global package_nr, last_line
    char = raw_stream.popleft()
    if char == c('P'):
        current_package_nr = ord(raw_stream.popleft())
        if (current_package_nr - package_nr + 256)% 256 != 1:
            endline()
            endline(['[PACKAGE LOST, last: ', str(package_nr),' new: ', str(current_package_nr), ' ]'])
        package_nr = current_package_nr
        l = [raw_stream.popleft() for i in range(0, package_len - 2)]
#        last_line = last_line + ['{']+l+['}']
        raw_buffer.extend(l)
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
    while len(raw_stream) > package_len:
        parse_raw_stream()
        while len(raw_buffer) > 5:
            parse_raw_buffer();





#while True:
#    uart_parse()
