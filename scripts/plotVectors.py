#!/usr/bin/python

__author__ = 'gdiaz'

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

class PlotVectors(object):
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self.n = 0
        self.O = [0, 0, 0]

    def setOrigin(self, x, y, z):
        self.O = [x, y, z]

    def config(self):
        mpl.rcParams['legend.fontsize'] = 10
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def plotAxes(self):
        self.ax.plot([self.O[0], self.O[0]+1], [self.O[1], self.O[1]+0], [self.O[2], self.O[2]+0], label='i')
        self.ax.plot([self.O[0], self.O[0]+0], [self.O[1], self.O[1]+1], [self.O[2], self.O[2]+0], label='j')
        self.ax.plot([self.O[0], self.O[0]+0], [self.O[1], self.O[1]+0], [self.O[2], self.O[2]+1], label='k')
        self.ax.legend()

    def plot(self, p):
        self.ax.plot([self.O[0], self.O[0]+p[0]], [self.O[1], self.O[1]+p[1]], [self.O[2], self.O[2]+p[2]], label='p'+str(self.n))
        self.ax.scatter(self.O[0]+p[0], self.O[1]+p[1], self.O[2]+p[2], c='k', marker='o')
        self.n += 1
        self.ax.legend()

    def show(self):
        plt.show()

if __name__ == '__main__':
    vectors = PlotVectors()
    #Test Example
    p1 = [1, 2, 3]
    p2 = [5, 5, 5]
    vectors.plotAxes()
    vectors.config()
    vectors.plot(p1)
    vectors.plot(p2)
    vectors.show()