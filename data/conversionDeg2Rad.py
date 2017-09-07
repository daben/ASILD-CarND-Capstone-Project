import numpy as np
import argparse
import math

parser = argparse.ArgumentParser(description='conversion 5th colomn from degrees to radians')
parser.add_argument('file', help='file name to convert');

args = parser.parse_args()


data = np.loadtxt(args.file,delimiter=',')
print(data)
print(data.shape)



data[:,3]=data[:,3]*math.pi/180.0

for y in range(0,data.shape[0]):
    while data[y,3]>=math.pi:
        data[y, 3] = data[y, 3] - 2*math.pi
    while data[y,3]<-math.pi:
        data[y, 3] = data[y, 3] + 2*math.pi


print(math.pi)

print(data)
print(data.dtype)
print(data.shape)


np.savetxt(args.file+'.radian.txt',data,delimiter=',',fmt='%.3f')
