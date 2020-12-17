
import numpy as np
from scipy.spatial.transform import Rotation as R 
import CalibUtil

n = 6
N = 500
v = 2

# ---------- User Definition ----------

# z-y-x
SA_gt = np.array([[ 20, -30,  65],
                  [ 40,  35, -10],
                  [  0, -20, -45],
                  [-30,   0,   0],
                  [-50,  60,  70]],dtype=np.float64)

RA_min = [5, 5, 5]
RA_max = [50, 60, 90]
RA_sig = [2, 2, 2]

# x-y-z
PA_gt = np.array([[80, 90, -90],
                  [-25, 68, 15],
                  [-16, 0, -55],
                  [33, -10, -33],
                  [0, 18, -52]],dtype=np.float64)

T_min = [5, 5, 5]
T_max = [100, 100, 100]
T_sig = np.array([[1, 1, 1],
                  [10, 7, 8],
                  [8, 5, 8],
                  [3, 2, 2],
                  [12, 8, 8],
                  [1, 5, 1]],dtype=np.float64)

scale = [5.7, 8.6, 1, 1, 1, 1]

# ---------- Prepare Groud Truth ----------

S_gt = np.zeros((3,3,n),dtype=np.float64)
for w in range(n-1):
    S_gt[:,:,w] = R.from_euler('zyx',SA_gt[w,:], degrees=True).as_matrix()

P_gt = np.zeros((3,1,n),dtype=np.float64)
for w in range(n-1):
    P_gt[:,:,w] = np.array([[PA_gt[w,0]],[PA_gt[w,1]],[PA_gt[w,2]]])

R_gt = np.zeros((3,3,N,n),dtype=np.float64)
T_gt = np.zeros((3,1,N,n),dtype=np.float64)

for k in range(N):

    RAz_gt = np.random.rand()*(RA_max[0]-RA_min[0]) + RA_min[0]
    RAy_gt = np.random.rand()*(RA_max[1]-RA_min[1]) + RA_min[1]
    RAx_gt = np.random.rand()*(RA_max[2]-RA_min[2]) + RA_min[2]

    Tx_gt = np.random.rand()*(T_max[0]-T_min[0]) + T_min[0]
    Ty_gt = np.random.rand()*(T_max[1]-T_min[1]) + T_min[1]
    Tz_gt = np.random.rand()*(T_max[2]-T_min[2]) + T_min[2]

    R_gt[:,:,k,0] = R.from_euler('zyx',[RAz_gt, RAy_gt, RAx_gt], degrees=True).as_matrix()
    T_gt[:,:,k,0] = np.array([[Tx_gt],[Ty_gt],[Tz_gt]])

    for w in range(n-1):
        R_gt[:,:,k,w+1] = np.matmul(np.matmul(S_gt[:,:,w].T, R_gt[:,:,k,w]), S_gt[:,:,w])
        T_gt[:,:,k,w+1] = np.matmul(S_gt[:,:,w].T, (np.matmul((R_gt[:,:,k,w]-np.eye(3)), P_gt[:,:,w]) + T_gt[:,:,k,w]))

print("groud truth OK, data size :",R_gt.shape,T_gt.shape)

# ---------- Add Noise and Scale ----------

for k in range(N):
    for w in range(n):

        R_delta = R.from_euler('zyx',[np.random.normal(0,RA_sig[0]), np.random.normal(0,RA_sig[1]), np.random.normal(0,RA_sig[2])], degrees=True).as_matrix()
        T_delta = np.array([[np.random.normal(0,T_sig[w,0])],[np.random.normal(0,T_sig[w,1])],[np.random.normal(0,T_sig[w,2])]])

        R_gt[:,:,k,w] = np.matmul(R_delta, R_gt[:,:,k,w])
        T_gt[:,:,k,w] = T_delta + T_gt[:,:,k,w]

print("add noise OK, data size :",R_gt.shape,T_gt.shape)

S = CalibUtil.SolveRN(R_gt)

P1 = CalibUtil.SolveTNWithoutClosure(R_gt,T_gt,S)
P2 = CalibUtil.SolveTNWithClosure(R_gt,T_gt,S)

for k in range(n-1):
    print("\n----------------------------------------")
    print("\n",k,k+1,"Trans ->")
    print("\n",PA_gt[k,0],PA_gt[k,1],PA_gt[k,2])
    print("\n",P1[3*k],P1[3*k+1],P1[3*k+2])
    print("\n",P2[3*k],P2[3*k+1],P2[3*k+2])



