import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import math


from motor import MOTOR
from sensor import SENSOR

class ROBOT:
    def __init__(self, solutionID):  
        self.solutionId = solutionID      
        self.robotId = p.loadURDF("body.urdf")
        self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        os.system("rm brain"+str(solutionID)+".nndf")
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
        for neuronName in self.nn.Get_Neuron_Names():
             if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)*c.motorJointRange
                self.motors[jointName].Set_Value(self.robotId, desiredAngle)
                
    def Think(self, i):
        CPG_Val = math.sin((c.PI/100)*i)*2*c.PI
        self.nn.Update(CPG_Val)
    
    def Get_Fitness(self):        
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]

        f = open("tmp"+str(self.solutionId)+".txt", "w")
        f.write(str(xPosition))
        f.close()
        os.system("mv tmp"+self.solutionId+".txt fitness"+self.solutionId+".txt")



        