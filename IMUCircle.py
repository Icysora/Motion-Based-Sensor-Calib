
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("AHRS1124_7.txt",dtype=float,comments="//",usecols=(1,2,3))

a1 = np.zeros((data.shape[0]))
a2 = np.zeros((data.shape[0]))
a3 = np.zeros((data.shape[0]))
v1 = np.zeros((data.shape[0]))
v2 = np.zeros((data.shape[0]))
v3 = np.zeros((data.shape[0]))
x1 = np.zeros((data.shape[0]))
x2 = np.zeros((data.shape[0]))
x3 = np.zeros((data.shape[0]))
L  = np.zeros((data.shape[0],3))
G  = np.zeros((data.shape[0],1))

dt = 1/400

for i in range(data.shape[0]):
    a1[i] = data[i,0]
    a2[i] = data[i,1]
    a3[i] = data[i,2]

i = 0
s1 = 0.
s2 = 0.
s3 = 0.

while abs(a1[i]) < 0.2: 
    s1 = s1 + a1[i]
    s2 = s2 + a2[i]
    s3 = s3 + a3[i]
    i = i + 1

a1b = s1 / i
a2b = s2 / i
a3b = s3 / i


for i in range(1,data.shape[0]):
    
    v1[i] = v1[i-1] + (a1[i-1] + a1[i] - 2*a1b) * dt / 2
    v2[i] = v2[i-1] + (a2[i-1] + a2[i] - 2*a2b) * dt / 2
    v3[i] = v3[i-1] + (a3[i-1] + a3[i] - 2*a3b) * dt / 2
    x1[i] = x1[i-1] + (v1[i-1] + v1[i]) * dt / 2
    x2[i] = x2[i-1] + (v2[i-1] + v2[i]) * dt / 2
    x3[i] = x3[i-1] + (v3[i-1] + v3[i]) * dt / 2

for i in range(data.shape[0]):
    L[i,0] = x1[i]
    L[i,1] = x2[i]
    L[i,2] = x3[i]
    G[i]   = 2 * (v1[i]*v1[i] + v2[i]*v2[i] + v3[i]*v3[i] + x1[i]*a1[i] + x2[i]*a2[i] + x3[i]*a3[i])

x = np.linalg.lstsq(L,G,rcond=-1)

print(x[0])

plt.plot(v3)
plt.show()

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(x1,x2,x3)
# plt.show()