import pybullet as p
import pybullet_data
import time as t

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)

p.loadSDF("boxes.sdf")
planeId = p.loadURDF("plane.urdf")

for i in range(0,1000):
    p.stepSimulation()
    t.sleep(1/100)
    print(i)
p.disconnect()
