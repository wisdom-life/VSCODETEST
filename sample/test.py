"""
测试目的：明确RTB—P下，利用DH参数展示机械臂的理论设计结果；
正运动学的计算及图示、逆运动学的计算结果、运动轨迹的显示。
"""

import spatialmath as smtb
import spatialmath.base.transforms3d as sbt3
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
import numpy as np #采用numpy进行计算
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# 定义DH参数下的XB

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=0.7161, a=0, alpha=0, offset=0),
        rtb.RevoluteMDH(d=0.43, a=0, alpha= pi/2, offset= pi),
        rtb.RevoluteMDH(d=0.43, a=0, alpha= pi/2, offset= pi/2),
        rtb.RevoluteMDH(d=0,a=2.080, alpha=0, offset=0,qlim=[0,0]),#增加虚拟关节,
        rtb.RevoluteMDH(d=0.387, a=0, alpha=0, offset=0),
        rtb.RevoluteMDH(d=0,a=2.080, alpha=0,offset=0,qlim=[0,0]),#增加虚拟关节
        rtb.RevoluteMDH(d=0.43, a=0, alpha=0, offset= pi/2),
        rtb.RevoluteMDH(d=0.43, a=0, alpha= pi/2,offset= pi),
        rtb.RevoluteMDH(d=0.7161, a=0, alpha= pi/2, offset= 0.0),
    ],
    name="xb",
)


Picture1 = rtb.backends.PyPlot.PyPlot()

''' //正常matplotlib显示
Picture1.launch()
Picture1.add(robot,
    readonly=False,
    display=True,
    jointaxes=True,
    eeframe=True,
    shadow=False,
    name=True
    )

'''

Picture1.launch("Teach " + robot.name)
Picture1.add(
    robot,
    readonly=True,
    jointaxes=True,
    jointlabels=True,
    eeframe=True,
    shadow=False,
    name=True,
    )


# robot.plot(q) #可以实现以下功能

robot.q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#from spatialmath.base.transforms3d import *
T = robot.fkine([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#trplot(T)
#sbt3.trplot(T)



Frame0 = SE3() # 明确基坐标系{0}

Frame0.plot(frame='0', color='green') #由于矩阵基类对相应的坐标系绘制，其画布的传递关系不明确，需注意该语句的位置。

#robot.plot([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

num_sample = 10000

q_range = np.array([pi,pi,pi,0,pi,0,pi,pi,pi])

rand_q = (np.random.rand(num_sample,9) - 0.5)* 2* q_range

MT = robot.fkine(rand_q) #计算不同的末端位置

# print(MT)
print(MT[0])
#print(MT[0].)
#MT[0].data[1:]


m_x = []
m_y = []
m_z = []

for item in range(num_sample):
    m_x.append(MT.data[item][0,3])
    m_y.append(MT.data[item][1,3])
    m_z.append(MT.data[item][2,3])

plt.gca().scatter3D(xs=m_x, ys=m_y, zs=m_z)



# MT.plot()

#plt.gca().scatter3D(xs=MT[0].x, ys=MT[0].y, zs=MT[0].z)

#print(rand_q)



Picture1._add_teach_panel(robot,robot.q)
  

Picture1.step()


Picture1.hold()



