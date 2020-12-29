
import numpy as np
import CalibDataProcessor
import CalibUtil
import matplotlib.pyplot as plt


odom1 = CalibDataProcessor.ReadDataFromFile('txt/leftodom_balm.txt',file_type='ROS',time_stamp='delta')
odom2 = CalibDataProcessor.ReadDataFromFile('txt/middleodom_balm.txt',file_type='ROS',time_stamp='delta')
odom3 = CalibDataProcessor.ReadDataFromFile('txt/rightodom_balm.txt',file_type='ROS',time_stamp='delta')

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(odom1[:,1],odom1[:,2],odom1[:,3],color='m')
# ax.plot(odom2[:,1],odom2[:,2],odom2[:,3],color='g')
# ax.plot(odom3[:,1],odom3[:,2],odom3[:,3],color='b')
# plt.show()

odom1s, odom2s = CalibDataProcessor.TimestampMatch(odom1, odom2, time_th=0.05)
R1A, R1B, T1A, T1B = CalibDataProcessor.PrepareAB(odom1s, odom2s, r_th=1.4, log=False)
RX, AX = CalibUtil.SolveR(R1A,R1B,name="Rml")

odom2s, odom3s = CalibDataProcessor.TimestampMatch(odom2, odom3, time_th=0.05)
R2B, R2C, T2B, T2C = CalibDataProcessor.PrepareAB(odom2s, odom3s, r_th=1.4, log=False)
RY, AY = CalibUtil.SolveR(R2B,R2C,name="Rrm")

odom3s, odom1s = CalibDataProcessor.TimestampMatch(odom3, odom1, time_th=0.05)
R3C, R3A, T3C, T3A = CalibDataProcessor.PrepareAB(odom3s, odom1s, r_th=1.4, log=False)
RZ, AZ = CalibUtil.SolveR(R3C,R3A,name="Rlr")

print(np.matmul(np.matmul(RX,RY),RZ))



# T3 = CalibUtil.SolveT3nz(R1A, R1B, R2B, R2C, R3C, R3A, T1A, T1B, T2B, T2C, T3C, T3A, RX, RY, RZ, log=True)

# TX = CalibUtil.SolveT(R1A, R1B, T1A, T1B, RX, ignore_z=True)

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(tx2,ty2,tz2)
# plt.show()





 
    

    
    






    




            








    


    


