
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rt 
from scipy.optimize import minimize

data = np.loadtxt("1202_3.txt",dtype=float,comments="//",usecols=(1,2,3,4,5,6))

print(">1")


plt.plot(data[:,3])
plt.plot(data[:,4])
plt.plot(data[:,5])
plt.show()
    



