import roboticstoolbox as rtb
from spatialmath import SE3
robot = rtb.models.DH.Panda()
print(robot)
T = robot.fkine(robot.qz)
print(T)
T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0],[0, 0, -1])
sol = robot.ikine_LM(T)
print(sol)
q_pickup = sol.q
print(robot.fkine(q_pickup))
qt = rtb.jtraj(robot.qz, q_pickup, 500)
# robot.plot(qt.q, movie='panda1.gif')

robot = rtb.models.URDF.Panda()
print(robot)
from roboticstoolbox.backends.swift import Swift
backend = Swift()
backend.launch()
backend.add(robot)
for qk in qt.q:
    robot.q = qk
    backend.step()