
import numpy as np
from scipy.spatial.transform import Rotation as Rot 
from scipy.optimize import minimize

def SolveR(RA, RB, max_size=2000, log=True, name="RX"):

    """
    Solve RA * RX = RX * RB
    """

    n = min(RA.shape[2],max_size)
    M = np.zeros((9*n,9),dtype=np.float64)

    for k in range(n):
        M[9*k:9*k+9,:] = np.kron(RA[:,:,k],np.eye(3)) - np.kron(np.eye(3),RB[:,:,k].T)

    u1, s1, v1 = np.linalg.svd(M)
    u2, s2, v2 = np.linalg.svd(v1[-1,:].reshape(3,3))
    RX = np.matmul(u2, v2)

    if np.linalg.det(RX) < 0:
        RX = -RX

    angleX = Rot.from_matrix(RX).as_euler('zyx',degrees=True)

    if(log):
        
        print("\n----------------------------------------\n")
        print(name,"->")
        print(RX)
        print("\nangle (zyx) (May not correct) ->")
        print(angleX)
        print("\n----------------------------------------\n")

    return RX, angleX

def SolveT(RA, RB, TA, TB, RX, scale=1, ignore_z=False, log=True):

    """
    Solve (RA - I) * TX + s * TA = RX * TB

    scale ≠ 0 -> s = scale
    scale = 0 -> s is unknown
    """

    n = RA.shape[2]
    
    if scale == 0:

        L = np.zeros((3*n,4),dtype=np.float64)
        P = np.zeros((3*n,1),dtype=np.float64)

        for k in range(n):
            L[3*k:3*k+3,0:3] = RA[:,:,k] - np.eye(3)
            L[3*k:3*k+3,3]   = TA[:,:,k].reshape(3)
            P[3*k:3*k+3,:]   = np.matmul(RX, TB[:,:,k])

        if ignore_z:
            TXresult = np.linalg.lstsq(L[:,0:3],P,rcond=-1)
        else:
            TXresult = np.linalg.lstsq(L,P,rcond=-1)

    else:
        
        L = np.zeros((3*n,3),dtype=np.float64)
        P = np.zeros((3*n,1),dtype=np.float64)

        for k in range(n):
            L[3*k:3*k+3,0:3] = RA[:,:,k] - np.eye(3)
            P[3*k:3*k+3,:]   = np.matmul(RX, TB[:,:,k]) - TA[:,:,k] * scale

        if ignore_z:
            TXresult = np.linalg.lstsq(L[:,0:2],P,rcond=-1)
        else:
            TXresult = np.linalg.lstsq(L,P,rcond=-1)

    if log:

        print("\n----------------------------------------")
        print("\nSingle Values of L ->")
        print(TXresult[3])
        print("\nTX / scale ->")
        print(TXresult[0])  
        print("\nLS error ->")
        print(TXresult[1])
        print("\n----------------------------------------")


    return TXresult[0]

def SolveT3nz(R1A, R1B, R2B, R2C, R3C, R3A, T1A, T1B, T2B, T2C, T3C, T3A, RX, RY, RZ, log=True):

    n1 = R1A.shape[2]
    n2 = R2B.shape[2]
    n3 = R3C.shape[2]

    L = np.zeros((3*(n1+n2+n3),6),dtype=np.float64)
    P = np.zeros((3*(n1+n2+n3),1),dtype=np.float64)

    for k in range(n1):
        L[3*k:3*k+3,0:3] = R1A[:,:,k] - np.eye(3)
        L[3*k:3*k+3,3:6] = np.zeros((3,3))
        P[3*k:3*k+3,:]   = np.matmul(RX,T1B[:,:,k]) - T1A[:,:,k]
    for k in range(n2):
        L[3*k+n1:3*k+3+n1,0:3] = np.zeros((3,3))
        L[3*k+n1:3*k+3+n1,3:6] = R2B[:,:,k] - np.eye(3)
        P[3*k+n1:3*k+3+n1,:]   = np.matmul(RY,T2C[:,:,k]) - T2B[:,:,k]
    for k in range(n3):
        L[3*k+n1+n2:3*k+3+n1+n2,0:3] = np.matmul(RZ,RY) - np.matmul(R3C[:,:,k],np.matmul(RZ,RY))
        L[3*k+n1+n2:3*k+3+n1+n2,3:6] = RZ - np.matmul(R3C[:,:,k],RZ)
        P[3*k+n1+n2:3*k+3+n1+n2,:]   = np.matmul(RZ,T3A[:,:,k]) - T3C[:,:,k]

    TXresult = np.linalg.lstsq(L,P,rcond=-1)

    if(log):

        print("\n----------------------------------------")
        print("\nSingle Values of L ->")
        print(TXresult[3])
        print("\nSolveT3 result ->")
        print(TXresult[0])
        print("\nLS error ->")
        print(TXresult[1])
        print("\n----------------------------------------")

    return TXresult[0]

def SolveRNCost(x, Rin):

    N = Rin.shape[2]
    n = Rin.shape[3]
    Cost = 0

    S = np.zeros((3,3,n-1),dtype=np.float64)

    for k in range(n-1):
        S[:,:,k] = Rot.from_euler('zyx',x[3*k:3*k+3],degrees=True).as_matrix()        

    for h in range(N):
        for i in range(1,n):
            for j in range(i+1,n+1):

                Rij = np.eye(3,dtype=np.float64)

                for k in range(i,j):
                    Rij = np.matmul(Rij,S[:,:,k-1])
                
                Cost = Cost + np.linalg.norm(np.matmul(Rin[:,:,h,i-1],Rij)-np.matmul(Rij,Rin[:,:,h,j-1])) 

    # print(Cost)       

    return Cost


def SolveRN(Rin, use_opt=False):

    """
            R1 * S1 = S1 * R2
            R2 * S2 = S2 * R3
    Solve   ...
            Rn-2 * Sn-2 = Sn-2 * Rn-1
            Rn-1 * Sn-1 = Sn-1 * Rn

    input size: 3 x 3 x N x n
    """

    n = Rin.shape[3]
    Sout = np.zeros((3,3,n-1),dtype=np.float64)
    xinit = np.zeros((3*n-3,),dtype=np.float64)

    for k in range(n-1):

        Sout[:,:,k],Aout = SolveR(Rin[:,:,:,k],Rin[:,:,:,k+1],log=False)
        print("\n",k,k+1,"R Init ->")
        print(Aout)
        xinit[3*k:3*k+3] = Aout

    if use_opt:

        res = minimize(SolveRNCost, xinit, args=Rin, options={'maxiter':50})

        for k in range(n-1):
            Sout[:,:,k] = Rot.from_euler('zyx',res.x[3*k:3*k+3],degrees=True).as_matrix()
            print("\n",k,k+1,"R ... ->")
            print(res.x[3*k:3*k+3])        

    return Sout

def SolveTNWithoutClosure(Rin, Tin, Sin):

    """
    Rin: 3 x 3 x N x n
    Tin: 3 x 1 x N x n
    Sin: 3 x 3 x (n-1)
    """

    n = Rin.shape[3]
    Pout = np.zeros((3*n-3,1),dtype=np.float64)

    for k in range(n-1):

        Pout[3*k:3*k+3,:] = SolveT(Rin[:,:,:,k],Rin[:,:,:,k+1],Tin[:,:,:,k],Tin[:,:,:,k+1],Sin[:,:,k],log=False)

    return Pout

def SolveTNWithClosure(Rin, Tin, Sin):

    """
    Rin: 3 x 3 x N x n
    Tin: 3 x 1 x N x n
    Sin: 3 x 3 x (n-1)
    """

    N = Rin.shape[2]
    n = Rin.shape[3]
    L = np.zeros((int(3*N*n*(n-1)/2),3*n-3),dtype=np.float64)
    G = np.zeros((int(3*N*n*(n-1)/2),1),dtype=np.float64)

    row_count = 0

    # Fill (Ri-I)*tij=Rij*tj-ti
    for h in range(N):
        for i in range(1,n):
            for j in range(i+1,n+1):
                
                #Left Side
                L[3*row_count:3*row_count+3,3*i-3:3*i] = Rin[:,:,h,i-1]-np.eye(3)
                for k in range(i,j-1):
                    Sk = np.eye(3,dtype=np.float64)
                    for m in range(i,k+1):
                        Sk = np.matmul(Sk, Sin[:,:,m-1])
                    Lk = np.matmul((Rin[:,:,h,i-1]-np.eye(3)),Sk)
                    L[3*row_count:3*row_count+3,3*k:3*k+3] = Lk

                #Right Side
                Rij = np.eye(3,dtype=np.float64)
                for k in range(i,j):
                    Rij = np.matmul(Rij,Sin[:,:,k-1])
                G[3*row_count:3*row_count+3,:] = np.matmul(Rij,Tin[:,:,h,j-1])-Tin[:,:,h,i-1]

                row_count = row_count + 1


    x = np.linalg.lstsq(L,G,rcond=-1)

    return x[0]











