
import numpy as np
from scipy.spatial.transform import Rotation as Rot 

def ReadDataFromFile(file_name, file_type='ORB', time_stamp="real", log='[ReadData]'):

    """
    file_type: 'ORB' or 'ROS'
    """
    
    if file_type == 'ORB':
        data = np.loadtxt(file_name,dtype=np.float64)

    if file_type == 'ROS':
        data = np.loadtxt(file_name,dtype=np.float64,comments='%',delimiter=',',usecols=(2,5,6,7,8,9,10,11))
        data[:,0] = data[:,0] / 1000000000

    if time_stamp == 'delta':
        data[:,0] = data[:,0] - data[0,0]

    print(log,'Read',data.shape[0],'data from',file_name)

    return data

def TimestampMatch(odom1, odom2, time_th=0.05, log='[TsMatch]'):

    """
    input:  odom1   (m x 8) = [ts1_1, tx1_1, ty1_1, tz1_1, rx1_1, ry1_1, rz1_1, rw1_1]
                              [ts1_2, tx1_2, ty1_2, tz1_2, rx1_2, ry1_2, rz1_2, rw1_2]
                              ...
                              [ts1_m, tx1_m, ty1_m, tz1_m, rx1_m, ry1_m, rz1_m, rw1_m] 

            odom2   (n x 8) = [ts2_1, tx2_1, ty2_1, tz2_1, rx2_1, ry2_1, rz2_1, rw2_1]
                              [ts2_2, tx2_2, ty2_2, tz2_2, rx2_2, ry2_2, rz2_2, rw2_2]
                              ...
                              [ts2_n, tx2_n, ty2_n, tz2_n, rx2_n, ry2_n, rz2_n, rw2_n]

    output: odom1s  (p x 7) = [tx1_1, ty1_1, tz1_1, rx1_1, ry1_1, rz1_1, rw1_1]
                              [tx1_2, ty1_2, tz1_2, rx1_2, ry1_2, rz1_2, rw1_2]
                              ...
                              [tx1_p, ty1_p, tz1_p, rx1_p, ry1_p, rz1_p, rw1_p]    

            odom2s  (p x 7) = [tx2_1, ty2_1, tz2_1, rx2_1, ry2_1, rz2_1, rw2_1]
                              [tx2_2, ty2_2, tz2_2, rx2_2, ry2_2, rz2_2, rw2_2]
                              ...
                              [tx2_p, ty2_p, tz2_p, rx2_p, ry2_p, rz2_p, rw2_p]

    method: two frame is matched if |ts1_i - ts2_j| < time_th
    """

    odom1s = np.zeros((1,7),dtype=np.float64)
    odom2s = np.zeros((1,7),dtype=np.float64)

    i = 0
    j = 0

    while (i < odom1.shape[0]) & (j < odom2.shape[0]):

        if (odom1[i,0]-odom2[j,0] < time_th) & (odom1[i,0]-odom2[j,0] > -time_th):

            odom1sp = odom1[i,1:8].reshape(1,7)
            odom2sp = odom2[j,1:8].reshape(1,7)

            odom1s = np.append(odom1s, odom1sp, axis=0)
            odom2s = np.append(odom2s, odom2sp, axis=0)

        if odom1[i,0] < odom2[j,0]:
            i = i + 1
        else:
            j = j + 1

    print(log,'Get',odom1.shape[0]-1,'data pairs after timestamp match')

    return odom1s[1:,:], odom2s[1:,:]

def PrepareAB(odom1, odom2, index_start=0, index_end=0, 
              index_delta=(20,30,40,50,70,100,130,160,200,240), index_step=1, 
              r_th=1.2, t_min=0.3, t_max=0.6, log='[GetAB]'):
 
    """
    ---------------------------------------------------------------------------------------------------------------------------------------
    input:  odom1/2(px7) =  [tx_1, ty_1, tz_1, rx_1, ry_1, rz_1, rw_1]
                            [tx_2, ty_2, tz_2, rx_2, ry_2, rz_2, rw_2]
                            ...
                            [tx_p, ty_p, tz_p, rx_p, ry_p, rz_p, rw_p]    

    output: RA/RB(3x3xk) =  [[r11_1, r12_1, r13_1]  [r11_2, r12_2, r13_2]       [r31_k, r32_k, r33_k]]
                            [[r21_1, r22_1, r23_1]  [r21_2, r22_2, r23_2]       [r31_k, r32_k, r33_k]]
                            [[r31_1, r32_1, r33_1], [r31_2, r32_2, r33_2], ..., [r31_k, r32_k, r33_k]]

            TA/TB(3x1xk) =  [[tx_1]  [tx_2]       [tx_k]]
                            [[ty_1]  [ty_2]       [ty_k]]
                            [[tz_1], [tz_2], ..., [tz_k]]

    method: |                   |                   |
            | <--------------(R1,T1)--------------- |
            |                   |                   |
            | <----(R0,T0)----- | ------(R,T)-----> |
            |                   |                   |
            init pose           pose at index k     pose at index k+w

            k is in range(index_start, data_size-index_end-w, index_step)
            w is in index_delta

            an (R,T) is valid if ||R-I|| > r_th (use 1-norm) and t_min < ||T|| < t_max (use 2-norm)
    ---------------------------------------------------------------------------------------------------------------------------------------
    """

    tx1 = odom1[:,0]
    ty1 = odom1[:,1]
    tz1 = odom1[:,2]
    rx1 = odom1[:,3]
    ry1 = odom1[:,4]
    rz1 = odom1[:,5]
    rw1 = odom1[:,6]
    tx2 = odom2[:,0]
    ty2 = odom2[:,1]
    tz2 = odom2[:,2]
    rx2 = odom2[:,3]
    ry2 = odom2[:,4]
    rz2 = odom2[:,5]
    rw2 = odom2[:,6]

    RA = np.zeros((3,3,1),dtype=np.float64)
    RB = np.zeros((3,3,1),dtype=np.float64)
    TA = np.zeros((3,1,1),dtype=np.float64)
    TB = np.zeros((3,1,1),dtype=np.float64)


    for w in index_delta:
        for k in range(index_start,tx1.size-index_end-w,index_step):

            RA1 = Rot.from_quat([rx1[k+w], ry1[k+w], rz1[k+w], rw1[k+w]]).as_matrix()
            RA0 = Rot.from_quat([rx1[k]  , ry1[k]  , rz1[k]  , rw1[k]  ]).as_matrix()
            RB1 = Rot.from_quat([rx2[k+w], ry2[k+w], rz2[k+w], rw2[k+w]]).as_matrix()
            RB0 = Rot.from_quat([rx2[k]  , ry2[k]  , rz2[k]  , rw2[k]  ]).as_matrix()

            RAP = np.matmul(RA1.T, RA0)
            RBP = np.matmul(RB1.T, RB0)
            
            if (np.linalg.norm(RAP-np.eye(3),ord=1)>=r_th) & (np.linalg.norm(RBP-np.eye(3),ord=1)>=r_th) :

                TAP = np.matmul(RA1.T, np.array([[tx1[k]-tx1[k+w]],[ty1[k]-ty1[k+w]],[tz1[k]-tz1[k+w]]]))
                TBP = np.matmul(RB1.T, np.array([[tx2[k]-tx2[k+w]],[ty2[k]-ty2[k+w]],[tz2[k]-tz2[k+w]]]))

                if (np.linalg.norm(TBP)>=t_min) & (np.linalg.norm(TBP)<=t_max):

                    RA = np.append(RA,RAP.reshape(3,3,1),axis=2)
                    RB = np.append(RB,RBP.reshape(3,3,1),axis=2)
                    TA = np.append(TA,TAP.reshape(3,1,1),axis=2)
                    TB = np.append(TB,TBP.reshape(3,1,1),axis=2)

                # print('k=',k,'\tw=',w,'--------------------------\n')
                # print('RA->\n',RAP)
                # print('RB->\n',RBP)
                # print('TA->\n',TAP)
                # print('TB->\n',TBP)
                # print('\n')

    print(log,'Get',RA.shape[2]-1,'valid A-B pairs for calibration')

    return RA[:,:,1:],RB[:,:,1:],TA[:,:,1:],TB[:,:,1:]









        



