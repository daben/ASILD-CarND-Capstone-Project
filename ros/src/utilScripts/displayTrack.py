import numpy as np

import matplotlib.pyplot as plt
import math

data = np.loadtxt('track2.csv')
print(data.shape)





dist = np.zeros(data.shape[0]-1)

for i in range(0,data.shape[0]-1):
    dist[i]=math.sqrt((data[i,0]-data[i+1,0])**2.0 + (data[i,1]-data[i+1,1])**2.0)

print("dist max between wps", np.max(dist))
print("dist mean between wps", np.mean(dist))


plt.plot(data[:, 0], data[:, 1],'.-')
plt.figure()

plt.plot(dist)
plt.show()
