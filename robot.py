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

        self.airborn = False
        self.jumps = 0
        self.timeAirborn = 0

        self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        os.system("rm brain"+str(solutionID)+".nndf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):
            self.sensors = {}
            for linkName in pyrosim.linkNamesToIndices:
                if (linkName[-5:] == "Lower"):
                    self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, i):
        numOff = 0
        for s in self.sensors:
            if (self.sensors[s].Get_Value(i) == -1):
                 numOff +=1

        if (numOff == 4):
             self.timeAirborn +=1
                
        if (numOff == 4 and (not self.airborn)):
            self.airborn = True
            self.jumps +=1

        if (numOff == 0 and self.airborn):
            self.airborn = False
            
            
    
    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
                self.motors[jointName] = MOTOR(jointName)
    
    def Act(self, i):
        for neuronName in self.nn.Get_Neuron_Names():
             if (self.nn.Is_Motor_Neuron(neuronName)):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                if (jointName != "Hidden"):
                    desiredAngle = self.nn.Get_Value_Of(neuronName)*c.motorJointRange
                    self.motors[jointName].Set_Value(self.robotId, desiredAngle)
                
    def Think(self, i):
        CPG_Val = math.sin((c.PI/100)*i)*2*c.PI
        self.nn.Update(CPG_Val)
    
    def Get_Fitness(self):        
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]

        fitness = self.timeAirborn

        f = open("tmp"+str(self.solutionId)+".txt", "w")
        f.write(str(fitness))
        f.close()
        os.system("mv tmp"+self.solutionId+".txt fitness"+self.solutionId+".txt")



        