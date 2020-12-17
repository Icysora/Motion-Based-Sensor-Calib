
import numpy as np

def cut(num,c):
    c=10**(-c)
    return (num//c)*c

data = np.loadtxt('lidarmiddle0921.txt',dtype=np.float64,comments='%',delimiter=',',usecols=(2,5,6,7,8,9,10,11))
data[:,0] = data[:,0] / 1000000000

for k in range(data.shape[0]):
    data[k,0] = cut(data[k,0],2)

np.savetxt('lidarmiddle0921orb.txt', data, fmt=['%.2f','%.14f','%.14f','%.14f','%.14f','%.14f','%.14f','%.14f'], delimiter=" ")
