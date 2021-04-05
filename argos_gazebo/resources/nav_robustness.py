#!/usr/bin/env python
"""plot the time data for world type"""
import sys
import math
import numpy as np 
import pandas as pd 
import matplotlib.pyplot as plt


b=[30,28,26,27,27,27,26,29,28,27,27,27,27,27,26,29,30,28,28,28,26,29,32,27,27,26,27,27,27,29,27,31,27,27,29,28,26,27,26,27,26,26,26,29,26,27,27,26,27,27,26,27,34,27,27,27,29,30,28,27,28,28,31,27,27,30,26,28,33,29,28,27,27,27,27,27,28,29,26,32,26,27,27,26,26,27,27,31,27,30,27,38,29,32,28]
c=[30,32,27,34,27,27,27,28,30,28,29,27,28,28,27,27,30,30,31,28,26,26,29,30,32,28,27,27,28,28,29,27,27,27,27,30,28,32,28,32,29,26,32,29,26,29,28,26,28,27,26,28,29,35,41,35,29,30,30,30,27,28,30,30,28,29,26,28,29,32,28,26,27,28,28,29,31,29,26,29,44,32,27,27,28,26,26,29,32,27,28,27,30,27,29]
d=[71,27,27,28,29,28,27,26,28,28,28,27,31,26,30,27,26,27,26,43,26,30,26,28,26,26,37,26,26,28,36,29,29,26,31,30,27,28,26,26,29,26,31,27,27,28,26,42,28,30,30,31,28,28,27,27,26,32,26,30,26,29,30,27,27,28,29,29,26,29,26,39,30,29,28,30,29,29,26,28,30,28,28,28,27,26,26,27,30,30]
print(np.mean(b),np.std(b))
print(np.mean(c),np.std(c))
print(np.mean(d),np.std(d))
fig, ax = plt.subplots(figsize=(20,20))
ax.set_title('Time to Reach Waypoint',fontsize=50)
boxprops = dict(linewidth=10)
meanlineprops = dict(linewidth=10, color='red')
ax.boxplot([b,c],boxprops=boxprops,meanprops=meanlineprops)
ax.set_xlabel('Terrain',fontsize=20)
ax.set_ylabel('Time[s]',fontsize=20)
ax.set_ylim(25,40)
ax.set_xticklabels(['Type B','Type C'],fontsize=20)
ax.set_yticks(np.arange(25, 40, 1))
tmp = np.linspace(25,40,16)
ax.set_yticklabels(tmp,fontsize=20)
ax.grid(lw=2,alpha=0.5)
fig.show()
fig.savefig('type_bc.png')