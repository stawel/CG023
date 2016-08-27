#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
from numpy import *
import numpy.linalg

import sys
import serial
import struct
import uartparser
#----------------------------------------------------
from numpy import cross, eye, dot
from scipy.linalg import expm3, norm

rt = [0.0]
rx = [0.0]
ry = [0.0]
rz = [0.0]
rp = [0.0]
ct = -1.0


def uart_parse():
    global ct
    uartparser.uart_parse_stream();
    while len(uartparser.line_buffer) > 0:
        line = uartparser.line_buffer.popleft()
        try:
            p = line.partition(" ")
            if p[0] == 'r:':
                v = [ float(x) for x in p[2].split() ]
                t = v[0]/1000000.
                #print ct, t
                if len(v) == 5 and ((ct < t and ct + 1. > t) or ct < 0):
                    ct = t
                    rt.append(t)
                    rx.append(v[1])
                    ry.append(v[2])
                    rz.append(v[3])
                    rp.append(v[4])
        except:
            print "parse exception"
            continue
#            raise



#------------------------------------------------------
fig1 = plt.figure()

l1, = plt.plot([], [], 'r-')
l2, = plt.plot([], [], 'g-')
l3, = plt.plot([], [], 'b-')
l4, = plt.plot([], [], 'y-')

plt.xlim(0., 100.)
plt.ylim(-1., 1.)


def update(num):
    uart_parse()
#    print "T:", rt
#    print "X:", rx
#    print ry

    l1.set_data(rt, rx)
    l2.set_data(rt, ry)
    l3.set_data(rt, rz)
    l4.set_data(rt, rp)


anim = animation.FuncAnimation(fig1, update, 25, interval=100, blit=False)


plt.show()

