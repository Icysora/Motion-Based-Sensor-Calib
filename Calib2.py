
import numpy as np
import CalibDataProcessor
import CalibUtil
from sys import argv
import yaml

if __name__ == "__main__":

    # Get Config Data
    yaml.warnings({'YAMLLoadWarning':False})
    yamlfile = open(argv[1],'r',encoding='utf-8')
    yamlcont = yamlfile.read()
    yamldata = yaml.load(yamlcont)
    yamlfile.close()

    # Get Data
    odom1 = CalibDataProcessor.ReadDataFromFile(yamldata['Odom_file'][0], log='[1/6]')
    odom2 = CalibDataProcessor.ReadDataFromFile(yamldata['Odom_file'][1], log='[2/6]')
    odom1s, odom2s = CalibDataProcessor.TimestampMatch(odom1, odom2,
                     time_th=yamldata['Ts_max'], log='[3/6]')
    RA, RB, TA, TB = CalibDataProcessor.PrepareAB(odom1s, odom2s,
                     #index_delta=range(2,50,1),
                     r_th=yamldata['DR_min'], t_min=yamldata['Dt_min'], t_max=yamldata['Dt_max'], log='[4/6]')

    # Calibration
    RX = CalibUtil.SolveR(RA, RB, group_size=yamldata['SVD_group_size'], log='[5/6]')
    TX = CalibUtil.SolveT(RA, RB, TA, TB, RX, scale=yamldata['Scale'], log='[6/6]')

    # Show Result
    print('\n--------------------\n')
    print('RX ->')
    print(RX)
    print('TX ->')
    print(TX)



 
    

    
    






    




            








    


    


