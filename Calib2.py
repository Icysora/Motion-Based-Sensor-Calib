
import numpy as np
import CalibDataProcessor
import CalibUtil
import matplotlib.pyplot as plt

odom1 = CalibDataProcessor.ReadDataFromFile('txt/1216-2-VLodom.txt',file_type='ROS',time_stamp='delta')
odom2 = CalibDataProcessor.ReadDataFromFile('txt/1216-2-RSodom.txt',file_type='ROS',time_stamp='delta')

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(odom1[:,1],odom1[:,2],odom1[:,3],color='m')
# ax.plot(odom2[:,1],odom2[:,2],odom2[:,3],color='b')
# plt.show()

odom1s, odom2s = CalibDataProcessor.TimestampMatch(odom1, odom2, time_th=0.05)
R1A, R1B, T1A, T1B = CalibDataProcessor.PrepareAB(odom1s, odom2s, r_th=1.4, t_th=0.1, log=False)
RX, AX = CalibUtil.SolveR(R1A,R1B,name="R")


TX = CalibUtil.SolveT(R1A, R1B, T1A, T1B, RX, ignore_z=False)






 
    

    
    






    




            








    


    


