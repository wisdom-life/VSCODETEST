"""
测试目的：明确RTB—P下，利用DH参数展示机械臂的理论设计结果；
正运动学的计算及图示、逆运动学的计算结果、运动轨迹的显示。
"""

import spatialmath as smtb
import spatialmath.base.transforms3d as sbt3
import roboticstoolbox as rtb
from math import pi


# 定义DH参数下的XB

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=0.7161, a=0, alpha=0, offset=0),
        rtb.RevoluteMDH(d=0.43, a=0, alpha= pi/2, offset= pi),
        rtb.RevoluteMDH(d=0.43, a=0, alpha= pi/2, offset= pi/2),
        rtb.RevoluteMDH(d=0.387, a=2.080, alpha=0, offset=0),
        rtb.RevoluteMDH(d=0.43, a=2.080, alpha=0, offset= pi/2),
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

robot.q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#from spatialmath.base.transforms3d import *
T = robot.fkine([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#trplot(T)
#sbt3.trplot(T)
T.plot() #由于矩阵基类对相应的坐标系绘制，其画布的传递关系不明确，需注意该语句的位置。

#robot.plot([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

Picture1._add_teach_panel(robot,robot.q)
  

Picture1.step()


Picture1.hold()



