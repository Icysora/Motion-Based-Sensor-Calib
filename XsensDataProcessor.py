
import numpy as np
from time import mktime
import argparse
import sys
import matplotlib.pyplot as plt

def ReadDataFromTXT(file_name, file_type='ORB'):

    """
    file_type: 'ORB' or 'ROS' or 'Xsens' or 'KITTI"
    """
    
    if file_type == 'ORB':
        data = np.loadtxt(file_name,dtype=np.float64)

    if file_type == 'ROS':
        data = np.loadtxt(file_name,dtype=np.float64,comments='%',delimiter=',',usecols=(2,5,6,7,8,9,10,11))
        data[:,0] = data[:,0] / 1e9

    if file_type == 'Xsens':
        data = np.genfromtxt(fname='Xsens_Example.txt', dtype=np.float64, comments="//", delimiter=",", 
                         missing_values='', filling_values=np.nan, usecols=(2,3,4,5,6,7,8,10,11,12,13,14,15))

        i = 0
        while i < data.shape[0]:
            if data[i,-1] != data[i,-1]:
                data = np.delete(data,i,axis=0)  
            else: 
                data[i,6] = mktime((int(data[i,1]),int(data[i,2]),int(data[i,3]),int(data[i,4]),int(data[i,5]),int(data[i,6]),-1,-1,-1)) + data[i,0]/1e9
                i = i + 1 
        data = data[:,6:]

    if file_type == 'KITTI':
        data = np.loadtxt(file_name,dtype=np.float64)

    return data

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-f','--file',help='input file names',nargs='+',required=True)
    parser.add_argument('-t','--type',help='input file types',nargs='+',required=True)

    args = parser.parse_args()
    filename = args.file
    filetype = args.type
    
    if len(filename) != len(filetype):
        print("Number of filename must equal to filetype !")
        sys.exit(0)

    validfiletype = ['ORB','ROS','Xsens','KITTI']

    for i in range(0,len(filename)):

        if filetype[i] not in validfiletype:
            print("Filetype",i+1,"is wrong. It must in",validfiletype)
            sys.exit(0)

        data = ReadDataFromTXT(filename[i],filetype[i])
        print("Origin Odometry",i,": Read",data.shape[0],"record in",filename[i],", type :",filetype[i])

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.plot(data[:,3],data[:,7],data[:,11])
    # plt.show()

