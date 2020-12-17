
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rt 
from scipy.optimize import minimize

def ba_cost(x, a, R, v0, dt):

    """
    x: a_bias, 1x3
    a: acc_ms, nx3
    R: gry_ms, nx3x3

    """

    v = np.zeros((a.shape[0],3))
    L = np.zeros((a.shape[0],3))
    B = np.zeros((a.shape[0],1))

    v[0,:] = v0
    for i in range(a.shape[0]):
        if i >=1:
            v[i,:] = v[i-1,:] + (a[i,:] + a[i-1,:] - 2*x) * dt / 2
        L[i,:] = np.matmul((a[i,:]-x),R[i,:,:])
        B[i,:] = v[i,0]*v[i,0] + v[i,1]*v[i,1] + v[i,2]*v[i,2]

    c = np.linalg.lstsq(L,B,rcond=-1)[0]

    return np.linalg.norm(np.matmul(L,c)-B)

def ba_cost_2(x, a, v0, s0, dt):

    v = np.zeros((a.shape[0],3))
    s = np.zeros((a.shape[0],3))
    L = np.zeros((a.shape[0],3))
    B = np.zeros((a.shape[0],1))

    v[0,:] = v0
    s[0,:] = s0
    for i in range(a.shape[0]):
        if i >=1:
            v[i,:] = v[i-1,:] + (a[i,:] + a[i-1,:] - 2*x) * dt / 2
            s[i,:] = s[i-1,:] + (v[i,:] + v[i-1,:]) * dt / 2
        L[i,:] = s[i,:]
        B[i,:] = s[i,0]*s[i,0] + s[i,1]*s[i,1] + s[i,2]*s[i,2]

    c = np.linalg.lstsq(L,B,rcond=-1)[0]

    return np.linalg.norm(np.matmul(L,c)-B)



data = np.loadtxt("./txt/1202_3.txt",dtype=float,comments="//",usecols=(1,2,3,4,5,6))

print(">1")

dt = 1/400
bn = 1200

a = np.zeros((data.shape[0],3))
R = np.zeros((data.shape[0],3,3))
b = np.zeros((data.shape[0],3))
v = np.zeros((data.shape[0],3))
x = np.zeros((data.shape[0],3))

for i in range(1,data.shape[0]-1):
    a[i,:] = np.array([(data[i-1,0]+data[i,0]+data[i+1,0])/3,
                       (data[i-1,1]+data[i,1]+data[i+1,1])/3,
                       (data[i-1,2]+data[i,2]+data[i+1,2])/3])

for i in range(data.shape[0]):
    R[i,:,:] = Rt.from_euler('zyx',[data[i,3],data[i,5],data[i,4]],degrees=True).as_matrix()

print(">2")


i = 0
s = 0.
while(np.linalg.norm(a[i,:])<0.15):
    s = s + a[i,:]
    i = i + 1

start_t = i
print("start_t:",start_t)

for k in range(i):
    b[k,:] = s / i

# for k in range(i,data.shape[0]):
#     b[k,:] = b[k-1,:]

while(i+bn < data.shape[0]):
    # b[i,:] = minimize(ba_cost, b[i-1,:], args=(a[i:i+bn,:],R[i:i+bn,:,:],v[i-1,:],dt)).x
    b[i,:] = minimize(ba_cost_2, b[i-1,:], args=(a[i-1:i+bn,:],v[i-1,:],x[i-1,:],dt)).x
    for k in range(i,i+bn): 
        b[k,:] = b[i,:]
        v[k,:] = v[k-1,:] + (a[k,:] + a[k-1,:] - b[k,:] - b[k-1,:]) * dt / 2
        x[k,:] = x[k-1,:] + (v[k,:] + v[k-1,:]) * dt / 2
    i = i + bn

final_t = i - bn
print(">3")




# plt.plot(b[:,1])
# plt.show()
    

QiuA = np.zeros((final_t-start_t,3))
QiuB = np.zeros((final_t-start_t,1))

for i in range(start_t,final_t):
    QiuA[i-start_t,:] = x[i,:] * 2
    QiuB[i-start_t,:] = x[i,0] * x[i,0] + x[i,1] * x[i,1] + x[i,2] * x[i,2]

QiuP = np.linalg.lstsq(QiuA,QiuB,rcond=-1)[0]
# QiuR = np.linalg.norm(QiuP)

# Qiuu = np.linspace(0, 2 * np.pi, 10)
# Qiuv = np.linspace(0, np.pi, 10)
# Qiux = QiuR * np.outer(np.cos(Qiuu), np.sin(Qiuv)) + QiuP[0]
# Qiuy = QiuR * np.outer(np.sin(Qiuu), np.sin(Qiuv)) + QiuP[1]
# Qiuz = QiuR * np.outer(np.ones(np.size(Qiuu)), np.cos(Qiuv)) + QiuP[2]


print(QiuP)

 
fig = plt.figure()
ax = fig.gca(projection='3d')
# ax.plot_surface(Qiux, Qiuy, Qiuz, color='m')
ax.plot(x[:,0],x[:,1],x[:,2],".")
plt.show()