import pybullet as p
import pybullet_data
import constants as c
import pyrosim.pyrosim as pyrosim
import time as t



from world import WORLD
from robot import ROBOT

class SIMULATION:
    def __init__(self, directOrGUI):
        #connect and such
        self.mode = directOrGUI
        if(self.mode == "DIRECT"):
            physicsClient = p.connect(p.DIRECT)
        else:
            physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,c.GRAVITY)
        
        self.world = WORLD()
        self.robot = ROBOT()
    
    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def Run(self):
        for i in range(0,1000):
            p.stepSimulation()

            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            if(self.mode != "DIRECT"):
                t.sleep(1/100)

def __del__(self):
    p.disconnect()