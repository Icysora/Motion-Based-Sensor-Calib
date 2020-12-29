
import numpy as np
import CalibDataProcessor
import CalibUtil
import yaml

if __name__ == "__main__":

    # Get Config Data
    yaml.warnings({'YAMLLoadWarning':False})
    yamlfile = open('CalibN.yaml','r',encoding='utf-8')
    yamlcont = yamlfile.read()
    yamldata = yaml.load(yamlcont)
    yamlfile.close()

    # Get Data
    ssnum = min(len(yamldata['Odom_file']), len(yamldata['Sensor_name']))   
    print('[1/6] Config read OK,',ssnum,'valid sensor odom files :')
    for i in range(ssnum):
        print('\t-',yamldata['Sensor_name'][i],':',yamldata['Odom_file'][i])

    odomdata = []    
    print('[2/6] Read data from odom file ...')
    for i in range(ssnum):
        odomdata.append(CalibDataProcessor.ReadDataFromFile(yamldata['Odom_file'][i], 
                        log='\t- '+yamldata['Sensor_name'][i]+' :', file_type='ROS'))

    odompairA = []
    odompairB = []
    print('[3/6] Timestamp match ...')
    for i in range(ssnum-1):
        odompA, odompB = CalibDataProcessor.TimestampMatch(odomdata[i], odomdata[i+1],
                         time_th=yamldata['Ts_max'], 
                         log='\t- '+yamldata['Sensor_name'][i]+'<'+yamldata['Sensor_name'][i+1]+' :')
        odompairA.append(odompA)
        odompairB.append(odompB)

    RA = []
    RB = []
    TA = []
    TB = []
    print('[4/6] Get AB ...')
    for i in range(ssnum-1):
        RAP, RBP, TAP, TBP = CalibDataProcessor.PrepareAB(odompairA[i], odompairB[i],
                             r_th=yamldata['DR_min'], t_min=yamldata['Dt_min'], t_max=yamldata['Dt_max'],
                             log='\t- '+yamldata['Sensor_name'][i]+'<'+yamldata['Sensor_name'][i+1]+' :')
        RA.append(RAP)
        RB.append(RBP)
        TA.append(TAP)
        TB.append(TBP)

    RX = []
    print('[5/6] Calculate R ...')
    for i in range(ssnum-1):
        RX.append(CalibUtil.SolveR(RA[i], RB[i], group_size=yamldata['SVD_group_size'], 
                  log='\t-', name='R['+yamldata['Sensor_name'][i]+'<'+yamldata['Sensor_name'][i+1]+']'))

    TX = []
    print('[6/6] Calculate T ... (TODO)')
    #TODO

    print('OK')
