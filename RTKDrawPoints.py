
import numpy as np
from folium import plugins
import folium
import os

print("loading data ...")
data_ori = np.loadtxt("txt/rtk1118_1.txt",dtype=np.float64,comments='//',usecols=(9,10,12,13))

N = data_ori.shape[0]
print("size:",N)

n = range(0,N,100)

m = folium.Map([data_ori[int(N/2),0], data_ori[int(N/2),1]], zoom_start=20)

location = data_ori[n,0:2]

folium.PolyLine(location).add_to(m)

m.save("rtk1118_1.html")  

print("OK")
