#!/usr/bin/env python

from math import pi, cos, sin
from random import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
def point(h, k, r):
    theta = random() * 2 * pi
    xy = h + cos(theta) * r, k + sin(theta) * r
    return [xy[0],xy[1]]

def antipodal(x,y,h,k):
	#center (h,k) 
	x2 = h-(x-h)
	y2 = k-(y-k)
	return [x2,y2]

xy = np.ndarray(shape=(100,2),dtype=float)
x2y2 = np.ndarray(shape=(100,2),dtype=float)
for i in range(0,100):
	xy[i] = point(0,0,10)
	x2y2[i] = antipodal(xy[i,0],xy[i,1],0,0)-xy[i]
print(x2y2)
df_start = pd.DataFrame({'x': xy[:, 0], 'y': xy[:, 1]})
df_end = pd.DataFrame({'x': x2y2[:, 0], 'y': x2y2[:, 1]})
df_start.to_excel("start.xlsx",sheet_name='Sheet_name_1')
df_end.to_excel("waypoints.xlsx",sheet_name='Sheet_name_1')
plt.scatter(*zip(*xy))
plt.scatter(*zip(*x2y2))
plt.grid(color='k', linestyle=':', linewidth=1)
plt.axes().set_aspect('equal', 'datalim')
plt.show()