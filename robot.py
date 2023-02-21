import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c

from motor import MOTOR
from sensor import SENSOR

class ROBOT:
    def __init__(self):        
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):
            self.sensors = {}
            for linkName in pyrosim.linkNamesToIndices:
                self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, i):
        for s in self.sensors:
            self.sensors[s].Get_Value(i)
    
    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
                self.motors[jointName] = MOTOR(jointName)
    
    def Act(self, i):
        for m in self.motors:
            self.motors[m].Set_Value(self.robotId, i)