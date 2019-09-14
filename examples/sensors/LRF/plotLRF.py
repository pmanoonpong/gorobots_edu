import numpy as np
import matplotlib.pyplot as plt
import sys

index, length = np.loadtxt("testLRF.dat",unpack=True, skiprows  =1)

length[length ==0] = 5700

plt.plot(index, length)
plt.show()