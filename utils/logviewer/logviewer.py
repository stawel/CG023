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

def M(axis, theta):
    return expm3(cross(eye(3), axis/norm(axis)*theta))


ori = array([[1.,0.,0.],[0.,1.,0.], [0.,0.,1.]])

def setOri():
    z = array([0.,0.,0.])
    setVector(1,z,ori[0])
    setVector(2,z,ori[1])
    setVector(3,z,ori[2])


def rotateOri(g):
    global ori
    ori =  dot(dot( M(ori[2],g[2]), M(ori[1],g[1])) , dot(M(ori[0],g[0]),ori))
#    print ori

def vToQ(v):
    return array([0.,v[0],v[1],v[2]])

def qToV(q):
    return array([q[1],q[2],q[3]])


def mulQ(r ,q):
    retu = array([0.,0.,0.,0.])
    retu[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3]
    retu[1] = r[0] * q[1] + r[1] * q[0] + r[2] * q[3] - r[3] * q[2]
    retu[2] = r[0] * q[2] - r[1] * q[3] + r[2] * q[0] + r[3] * q[1]
    retu[3] = r[0] * q[3] + r[1] * q[2] - r[2] * q[1] + r[3] * q[0]
    return retu

def qCon(q):
    return array([q[0], -q[1], -q[2], -q[3]])

def rotate(v, q):
    qv = vToQ(v)
    q2 = mulQ(q,qv)
    q_1 = qCon(q)
    q3 = mulQ(q2, q_1)
    r = qToV(q3)
#    print "qv: ", qv
#    print "q : ", q
#    print "q2: ", q2
#    print "q_1: ", q_1
#    print "q3: ", q3
#    print "r:", r
    return r


def rotateQ(q):
    global ori
#    print "q: ", q
    ori[0] = rotate(array([1.,0.,0.]), q)
    ori[1] = rotate(array([0.,1.,0.]), q)
    ori[2] = rotate(array([0.,0.,1.]), q)
#    orin = v3d_normalize(ori)
#    print "q: ", q
#    print "ori: ", ori


def uart_parse():
    uartparser.uart_parse_stream();
    while len(uartparser.line_buffer) > 0:
        line = uartparser.line_buffer.popleft()
        a = array([0,0,0])
        g = array([0,0,0])
        q = array([1.,0,0,0])
        q_read = False
        try:
            p = line.partition(" ")
            if p[0] == '6ax:':
                v = [ float(x) for x in p[2].split() ]
                if len(v) == 6:
                    g = array(v[0:3])/200.
                    if(norm(a) > 100 or norm(g) > 100):
                         raise ValueError('A very specific bad thing happened')
#                    print 'wynik:', v3d_length(g), v3d_length(a)
                    setVector(0,array([0,0,0]),a)
#                    setVector(1,array([0,0,0]),g)
            elif p[0] == 'G:':
                v = [ float(x) for x in p[2].split() ]
                if len(v) == 3:
                    G = array(v[0:3])/2000.
                    setVector(4, array([0,0,0]), G)

            elif p[0] == 'Q:':
                v = [ float(x) for x in p[2].split() ]
                if len(v) == 4:
                    q = array(v[0:4])
                    q_read = True

            elif p[0] == '3ax:':
                v = [ float(x) for x in p[2].split() ]
                if len(v) == 3:
                    a = array(v[0:3])/2000.
                    setVector(0,array([0,0,0]),a)

        except:
            print "parse exception"
            continue
#            raise

        #rotateOri(g);
        if q_read:
            rotateQ(q)
            setOri()


#------------------------------------------------------
fig1 = plt.figure()
ax = p3.Axes3D(fig1)
vectors3d = [ax.plot(array([0,0]),array([0,0]),array([0,0]),color)[0] for color in ['y','g', 'r', 'b', 'k' ]]
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

def v3d_normalize(a):
    return a/sqrt(v3d_length(a))


def update(num):
    #TODO
    uart_parse()


anim = animation.FuncAnimation(fig1, update, 25, interval=100, blit=False)


plt.show()

