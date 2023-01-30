import pybullet as p
import pybullet_data
import time as t

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")


while(True):
    p.stepSimulation()
    t.sleep(1/100)
    print(i)
p.disconnect()
