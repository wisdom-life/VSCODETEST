from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np
from roboticstoolbox.robot.DHLink import RevoluteDH
from spatialmath import SE3
import roboticstoolbox as rtb
from spatialmath.base import *




T =  transl(0.5,0.0,0.0) @ rpy2tr(0.1, 0.2,0.3, order='xyz') @ trotx(-90,'deg')
print(T)
T = SE3(0.5, 0.0, 0.0) * SE3.RPY([0.1, 0.2, 0.3], order='xyz') * SE3.Rx(-90, unit='deg')
print(T)
T.eul()
T.R
T.plot(color='red', frame='2')

robot = rtb.models.DH.Panda()
print(robot)

TT = robot.fkine(robot.qz)
print(TT)

puma = rtb.models.DH.Puma560()
puma.plot(puma.qr)
#tau = puma.rne(puma.qn,np.zeros((6,)),np.zeros((6,)))
#print(tau)




#x = np.linspace(0,2*np.pi,500)
#y = np.sin(x)
#R = SE3.Rx(np.linspace(0, np.pi/2, num=100))

#print(len(R))

#plt.fill(x,y,color="cornflowerblue",alpha = 0.4)
#plt.plot(x,y,color="cornflowerblue",alpha = 0.8)

plt.show()
msg = "Hello World"
print(msg)
