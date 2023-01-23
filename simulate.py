import pybullet as p
import time as t

physicsClient = p.connect(p.DIRECT)
for i in range(0,100):
    p.stepSimulation()
    t.sleep(1/100)
    print(i)
p.disconnect()
