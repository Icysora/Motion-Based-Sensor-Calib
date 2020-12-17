
import numpy as np
from scipy.spatial.transform import Rotation as R 
import CalibUtil

N = 1000

aXAB = [50, -60, 20]
aXBC = [-25, 30, 80]
tXAB = [2.3, 1.6, 1.9]
tXBC = [-0.3, -1.9, 3.3]
asigma = 3
tsigma = 0.02

RA = np.zeros((3,3,N),dtype=np.float64)
RB = np.zeros((3,3,N),dtype=np.float64)
RC = np.zeros((3,3,N),dtype=np.float64)

TA = np.zeros((3,1,N),dtype=np.float64)
TB = np.zeros((3,1,N),dtype=np.float64)
TC = np.zeros((3,1,N),dtype=np.float64)

for k in range(N):

    RA[:,:,k] = R.from_euler('zyx',[0, np.random.rand()*20, np.random.rand()*360], degrees=True).as_matrix()
    TA[0:3,:,k] = np.random.rand(3,1)*10

    RXAB = R.from_euler('zyx', [aXAB[0],aXAB[1],aXAB[2]], degrees=True).as_matrix()
    RXBC = R.from_euler('zyx', [aXBC[0],aXBC[1],aXBC[2]], degrees=True).as_matrix()
    TXAB = np.array([[tXAB[0]],[tXAB[1]],[tXAB[2]]])
    TXBC = np.array([[tXBC[0]],[tXBC[1]],[tXBC[2]]])

    RB[:,:,k] = np.matmul(np.matmul(RXAB.T, RA[:,:,k]), RXAB)   
    RC[:,:,k] = np.matmul(np.matmul(RXBC.T, RB[:,:,k]), RXBC) 
    TB[:,:,k] = np.matmul(RXAB.T, (np.matmul((RA[:,:,k]-np.eye(3)), TXAB) + TA[:,:,k]))
    TC[:,:,k] = np.matmul(RXBC.T, (np.matmul((RB[:,:,k]-np.eye(3)), TXBC) + TB[:,:,k]))

    RA[:,:,k] = np.matmul(R.from_euler('zyx',[np.random.normal(0,asigma),np.random.normal(0,asigma),np.random.normal(0,asigma)],degrees=True).as_matrix(),RA[:,:,k])
    RB[:,:,k] = np.matmul(R.from_euler('zyx',[np.random.normal(0,asigma),np.random.normal(0,asigma),np.random.normal(0,asigma)],degrees=True).as_matrix(),RB[:,:,k])
    RC[:,:,k] = np.matmul(R.from_euler('zyx',[np.random.normal(0,asigma),np.random.normal(0,asigma),np.random.normal(0,asigma)],degrees=True).as_matrix(),RC[:,:,k])

    TA[:,:,k] = TA[:,:,k] + np.array([[np.random.normal(0,tsigma)],[np.random.normal(0,tsigma)],[np.random.normal(0,tsigma)]])
    TB[:,:,k] = TB[:,:,k] + np.array([[np.random.normal(0,tsigma)],[np.random.normal(0,tsigma)],[np.random.normal(0,tsigma)]])
    TC[:,:,k] = TC[:,:,k] + np.array([[np.random.normal(0,tsigma)],[np.random.normal(0,tsigma)],[np.random.normal(0,tsigma)]])

                

RX = CalibUtil.SolveR(RA, RB)
RY = CalibUtil.SolveR(RB, RC)
RZ = CalibUtil.SolveR(RC, RA)

# TX = CalibUtil.SolveT(RA, RB, TA, TB, RX, need_scale=False)
# TY = CalibUtil.SolveT(RB, RC, TB, TC, RY, need_scale=False)
# TZ = CalibUtil.SolveT(RC, RA, TC, TA, RZ, need_scale=False)

# T3 = CalibUtil.SolveT3(RA, RB, RC, TA, TB, TC, RX, RY, RZ)

# Rm = np.matmul(np.matmul(RX,RY),RZ)
# Tm = np.matmul(RX,np.matmul(RY,TZ)+TY)+TX
# print(Rm)
# print(Tm)