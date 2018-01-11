#!/usr/bin/python

__author__ = 'gdiaz'

import matplotlib as mpl
from plotVectors import PlotVectors
import numpy as np

class Rotation(object):
    def __init__(self):
        self.vectors = PlotVectors()
        self.a = [0, 0, 0]

    def rotate_z(self, a, yaw):
        Az = np.matrix([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        a_ = np.matrix([[a[0]],
                        [a[1]],
                        [a[2]]])
        u = Az*a_
        return [u.item(0), u.item(1), u.item(2)]

    def rotate_frame_z(self, I, J, K, yaw):
        Az = np.matrix([[np.cos(yaw), np.sin(yaw), 0],
                        [-np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        I_ = np.matrix([I[0], I[1], I[2]])
        J_ = np.matrix([J[0], J[1], J[2]])
        K_ = np.matrix([K[0], K[1], K[2]])
        
        i_ = I_*Az
        j_ = J_*Az
        k_ = K_*Az

        i = [i_.item(0), i_.item(1), i_.item(2)]
        j = [j_.item(0), j_.item(1), j_.item(2)]
        k = [k_.item(0), k_.item(1), k_.item(2)]

        return [i, j, k]

    def vectorRotationTest(self):
        # Calcs
        p1 = [2, 0, 0]
        yaw = 90*np.pi/180
        p1_rot = self.rotate_z(p1, yaw)
        print p1_rot
        # Plot
        self.vectors.plotAxes()
        self.vectors.config()
        self.vectors.plot(p1)
        self.vectors.plot(p1_rot)
        self.vectors.show()

    def frameRotationTest(self):
        # Calcs
        I = [1, 0, 0]
        J = [0, 1, 0]
        K = [0, 0, 1]
        yaw = 45*np.pi/180
        ijk = self.rotate_frame_z(I, J, K, yaw)
        print ijk
        # Plot
        self.vectors.plotAxes()
        self.vectors.config()
        self.vectors.plot(ijk[0])
        self.vectors.plot(ijk[1])
        self.vectors.plot(ijk[2])
        self.vectors.show()

    def get_qT(self, yawT): #Return quaternion target given yaw target
        AT = np.matrix([[np.cos(yawT), np.sin(yawT), 0],
                        [-np.sin(yawT), np.cos(yawT), 0],
                        [0, 0, 1]])

        q4 = 0.5*np.sqrt(1+AT[0,0]+AT[1,1]+AT[2,2])
        q1 = 0.25*(AT[1,2]-AT[2,1])/q4
        q2 = 0.25*(AT[2,0]-AT[0,2])/q4
        q3 = 0.25*(AT[0,1]-AT[1,0])/q4

        return [q4, q1, q2, q3]

    def get_qE_(self, qT, qS):
        qT_ = np.matrix([[qT[0], qT[3], -qT[2], qT[1]],
                        [-qT[3], qT[0], qT[1], qT[2]],
                        [qT[2], -qT[1], qT[0], qT[3]],
                        [-qT[1], -qT[2], -qT[3], qT[0]]])

        qS_ = np.matrix([[-qS[1]],
                        [-qS[2]],
                        [-qS[3]],
                        [qS[0]]])

        qE = qT_*qS_

        return [qE.item(0), qE.item(1), qE.item(2), qE.item(3)]

    def get_qE(self, yawT, qS):
        qT = self.get_qT(yawT)
        qE = self.get_qE_(qT, qS)
        return qE

if __name__ == '__main__':
    rotation = Rotation()
    # Test Example
    # rotation.vectorRotationTest()
    rotation.frameRotationTest()