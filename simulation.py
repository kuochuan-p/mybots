import pybullet as p
import pybullet_data
import constants as c
import pyrosim.pyrosim as pyrosim
import time as t



from world import WORLD
from robot import ROBOT

class SIMULATION:
    def __init__(self):
        #connect and such
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,c.GRAVITY)
        
        self.world = WORLD()
        self.robot = ROBOT()

    def Run(self):
        for i in range(0,1000):
            p.stepSimulation()

            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)

            t.sleep(1/100)

def __del__(self):
    p.disconnect()