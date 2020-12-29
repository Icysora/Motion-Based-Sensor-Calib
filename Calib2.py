
import numpy as np
import CalibDataProcessor
import CalibUtil
import yaml

if __name__ == "__main__":

    # Get Config Data
    yaml.warnings({'YAMLLoadWarning':False})
    yamlfile = open('Calib2.yaml','r',encoding='utf-8')
    yamlcont = yamlfile.read()
    yamldata = yaml.load(yamlcont)
    yamlfile.close()

    # Get Data
    odom1 = CalibDataProcessor.ReadDataFromFile(yamldata['Odom_file'][0], log='[1/5]')
    odom2 = CalibDataProcessor.ReadDataFromFile(yamldata['Odom_file'][1], log='[1/5]')
    odom1s, odom2s = CalibDataProcessor.TimestampMatch(odom1, odom2,
                     time_th=yamldata['Ts_th'], log='[2/5]')
    RA, RB, TA, TB = CalibDataProcessor.PrepareAB(odom1s, odom2s,
                     r_th=yamldata['DR_th'], t_th=yamldata['Dt_th'], log='[3/5]')

    # Calibration
    RX = CalibUtil.SolveR(RA, RB, group_size=yamldata['SVD_group_size'], log='[4/5]')
    TX = CalibUtil.SolveT(RA, RB, TA, TB, RX, scale=yamldata['Scale'], log='[5/5]')

    # Show Result
    print('\n--------------------\n')
    print('RX ->')
    print(RX)
    print('TX ->')
    print(TX)



 
    

    
    






    




            








    


    


