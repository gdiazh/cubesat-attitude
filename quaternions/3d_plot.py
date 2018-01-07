import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')

x0=y0=z0=0

i = [[x0, 1], [y0, 0], [z0, 0]]
j = [[x0, 0], [y0, 1], [z0, 0]]
k = [[x0, 0], [y0, 0], [z0, 1]]

p1 = [[x0, 5], [y0, 5], [z0, 5]]
p2 = [[x0, 1], [y0, 2], [z0, 3]]

ax.plot(i[0], i[1], i[2], label='i')
ax.plot(j[0], j[1], j[2], label='j')
ax.plot(k[0], k[1], k[2], label='k')

ax.plot(p1[0], p1[1], p1[2], label='p1')
ax.scatter(p1[0][1], p1[1][1], p1[2][1], c='k', marker='o')

ax.plot(p2[0], p2[1], p2[2], label='p2')
ax.scatter(p2[0][1], p2[1][1], p2[2][1], c='k', marker='o')

ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
