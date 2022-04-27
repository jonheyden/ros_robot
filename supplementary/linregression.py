# -*- coding: utf-8 -*-
"""
Created on Tue Apr 19 13:56:34 2022

@author: jonhe
"""

import numpy as np
import matplotlib.pyplot as plt

encoder_speed = [0,12,24]
commanded_speed = [33,70,120]
eq1 = np.polyfit(encoder_speed, commanded_speed, 1)
x1 = 1


#first = eq1[0]*x1 + eq1[1]

print(eq1)

'''
poly1d_fn = np.poly1d(eq1) 
 
plt.plot(f1,d1, 'yo', f1, poly1d_fn(f1))
plt.xlim(0, 150)
plt.ylim(0, 30)
plt.show()

'''