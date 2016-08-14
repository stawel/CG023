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


def uart_parse():
    uartparser.uart_parse_stream();
    while len(uartparser.line_buffer) > 0:
        line = uartparser.line_buffer.popleft()
        print line
        try:
            p = line.partition(" ")
            if p[0] == '6ax:':
                v = [ float(x) for x in p[2].split() ]
                if len(v) == 6:
                    g = array(v[0:3])
                    a = array(v[3:6])/2000.
                    print 'wynik:', v3d_length(g), v3d_length(a)
                    setVector(0,array([0,0,0]),g)
                    setVector(1,array([0,0,0]),a)

        except:
            print "parse exception"
            pass
#            raise


#------------------------------------------------------
fig1 = plt.figure()
ax = p3.Axes3D(fig1)
vectors3d = [ax.plot(array([0,1]),array([0,1]),array([0,1]),color)[0] for color in ['r','g']]
#dot3d, = plt.plot([0.5],[0.5],'g.')


limit = 2.

ax.set_xlim3d([-limit, limit])
ax.set_xlabel('X')

ax.set_ylim3d([-limit, limit])
ax.set_ylabel('Y')

ax.set_zlim3d([-limit, limit])
ax.set_zlabel('Z')


def setVector(idx, a, b):
    v = array([a, b])
    vT= v.T
    vectors3d[idx].set_data(vT[0],vT[1])
    vectors3d[idx].set_3d_properties(vT[2])

def v3d_length(a):
    return numpy.linalg.norm(a)

def v3d_sqrt(a):
    return a/np.sqrt(v3d_length(a))


def update(num):
    #TODO
    uart_parse()


anim = animation.FuncAnimation(fig1, update, 25, interval=100, blit=False)


plt.show()

