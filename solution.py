import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import copy
import time
import constants as c
import math


class SOLUTION:


    def __init__(self, ident, variant, weights=None, ) -> None:
        self.variant = variant
        self.myID = ident

        if(weights):
            self.hiddenWeights = weights["Hidden"]
            self.cpgWeights = weights["CPG"]
            self.motorWeights = weights["Motor"]
        
        self.Create_Robot(weights)


    
    def Set_ID(self, ident):
        self.myID = ident

    def Evaluate(self, dOrG):
        os.system("python3 simulate.py " + dOrG + " " + str(self.myID) + " 2&>1 &")

        while not os.path.exists("fitness"+str(self.myID)+".txt"):
            time.sleep(0.01)
        f = open("fitness"+str(self.myID)+".txt", "r")
        
        fitness = f.read()
        os.system("rm fitness"+str(self.myID)+".txt")
        print(fitness)
        f.close()
        self.fitness = float(fitness)

    def Start_Simulation(self, dOrG):
        self.Create_Robot()
        os.system("python3 simulate.py " + dOrG + " " + str(self.myID) + " &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness"+str(self.myID)+".txt"):
            time.sleep(0.01)
        f = open("fitness"+str(self.myID)+".txt", "r")
        
        fitness = f.read()
        f.close()
        self.fitness = float(fitness)
        os.system("rm fitness"+str(self.myID)+".txt")

    def Mutate(self):
        #chose which set of weights to modify
        flip = random.randint(0,2)

        if (flip == 0):
            if(self.variant):
                #chose which set of hidden weights to modify(front or back)
                flip2 = random.randint(0,1)
                if(flip == 0):
                    randRow = random.randint(0,self.numSensors/2 - 1)
                    randCol = random.randint(0,self.numHidden/2-1)
                else:
                    randRow = random.randint(self.numSensors/2, self.numSensors- 1)
                    randCol = random.randint(self.numHidden/2, self.numHidden-1)
            else:          
                randRow = random.randint(0,self.numSensors-1)
                randCol = random.randint(0,self.numHidden-1)

            self.hiddenWeights[randRow][randCol] = random.random()*2-1
        elif(flip == 1):
            randRow = random.randint(0,self.numHidden-1)
            randCol = random.randint(0,self.numMotors-1)

            self.motorWeights[randRow][randCol] = random.random()*2-1
        else:
            randRow = random.randint(0,self.numCPG-1)
            randCol = random.randint(0,self.numHidden-1)

            self.cpgWeights[randRow][randCol] = random.random()*2-1

    def Save(self):
        f = open("opt"+str(self.myID)+".txt", "w")
        f.write("Hidden weights:\n")
        for currentRow in range(0,self.numSensors):
            for currentColumn in range(0,self.numHidden):
                f.write(str(self.hiddenWeights[currentRow][currentColumn]))
                f.write(", ")
            f.write("\n")

        f.write("CPG weights:\n")
        for currentRow in range(0,self.numCPG):
            for currentColumn in range(0,self.numHidden):
                f.write(str(self.cpgWeights[currentRow][currentColumn]))
                f.write(", ")
            f.write("\n") 

        f.write("Motor weights:\n")
        for currentRow in range(0,self.numHidden):
            for currentColumn in range(0,self.numMotors):
                f.write(str(self.motorWeights[currentRow][currentColumn]))
                f.write(", ")
            f.write("\n")
        
        f.close()

           

    def Create_world(self):

        pyrosim.Start_SDF("world.sdf")
        length, width, height = 1,1,1
        x,y,z = 0,0,0.5
        pyrosim.End()

    def Generate_Body(self):

        pyrosim.Start_URDF("body.urdf")
        length, width, height = 3,2,.5
        x,y,z = 0,0,2.5
        
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])

        #back legs
        pyrosim.Send_Joint( name = "Torso_BackRightLeg" , parent= "Torso" , child = "BackRightLeg" , type = "revolute", position = [length/2,width/2,z-.5*(height)], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackRightLeg", pos=[0,0,-.5] , size=[.2,0.2,1])
        pyrosim.Send_Joint( name = "Torso_BackLeftLeg" , parent= "Torso" , child = "BackLeftLeg" , type = "revolute", position = [length/2,-width/2,z-.5*(height)], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackLeftLeg", pos=[0,0,-.5] , size=[.2,0.2,1])

        pyrosim.Send_Joint( name = "BackRightLeg_BackRightLower" , parent= "BackRightLeg" , child = "BackRightLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackRightLower", pos=[0,0,-.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint( name = "BackLeftLeg_BackLeftLower" , parent= "BackLeftLeg" , child = "BackLeftLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackLeftLower", pos=[0,0,-.5] , size=[0.2,0.2,1])

        #front legs
        pyrosim.Send_Joint( name = "Torso_FrontRightLeg" , parent= "Torso" , child = "FrontRightLeg" , type = "revolute", position = [-length/2,width/2,z-.5*(height)], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontRightLeg", pos=[0,0,-.5] , size=[.2,0.2,1])
        pyrosim.Send_Joint( name = "Torso_FrontLeftLeg" , parent= "Torso" , child = "FrontLeftLeg" , type = "revolute", position = [-length/2,-width/2,z-.5*(height)], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontLeftLeg", pos=[0,0,-.5] , size=[.2,0.2,1])

        pyrosim.Send_Joint( name = "FrontRightLeg_FrontRightLower" , parent= "FrontRightLeg" , child = "FrontRightLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontRightLower", pos=[0,0,-.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint( name = "FrontLeftLeg_FrontLeftLower" , parent= "FrontLeftLeg" , child = "FrontLeftLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontLeftLower", pos=[0,0,-.5] , size=[0.2,0.2,1])

        

        pyrosim.End()

    def Generate_Brain(self, weights=None):

        pyrosim.Start_URDF("brain"+str(self.myID)+".nndf")

        self.numNeurons = 0
        self.numSensors = 0
        self.numHidden = 0
        self.numMotors = 0
        self.numCPG = 0

        #SENSORS
        pyrosim.Send_Sensor_Neuron(name = self.numNeurons , linkName = "BackRightLower")
        self.numNeurons +=1
        self.numSensors +=1
        pyrosim.Send_Sensor_Neuron(name = self.numNeurons , linkName = "BackLeftLower")
        self.numNeurons +=1
        self.numSensors +=1
        pyrosim.Send_Sensor_Neuron(name = self.numNeurons , linkName = "FrontRightLower")
        self.numNeurons +=1
        self.numSensors +=1
        pyrosim.Send_Sensor_Neuron(name = self.numNeurons , linkName = "FrontLeftLower")
        self.numNeurons +=1
        self.numSensors +=1
        

        
        #HIDDEN
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Hidden")
        self.numNeurons +=1
        self.numHidden +=1

        if(not weights):
            self.hiddenWeights = (np.random.rand(self.numSensors, self.numHidden))*2-1
        
        if(self.variant):
            for backSensorNeuron in range(0,2):
                for frontHiddenNeuron in range(4,8):
                    self.hiddenWeights[backSensorNeuron, frontHiddenNeuron] = 0
            for frontSensorNeuron in range(2,4):
                for backHiddenNeuron in range(0,4):
                    self.hiddenWeights[frontSensorNeuron, backHiddenNeuron] = 0

        #add synapses from sensors to hidden
        for currentRow in range(0,self.numSensors):
            for currentColumn in range(0,self.numHidden):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow  , targetNeuronName = currentColumn+self.numSensors , weight = self.hiddenWeights[currentRow][currentColumn] )

        #CPG
        pyrosim.Send_Sensor_Neuron(name = self.numNeurons , linkName = "CPG")
        self.numNeurons +=1
        self.numCPG +=1

        if(not weights):
            self.cpgWeights = (np.random.rand(self.numCPG, self.numHidden))*2-1
        
        for currentRow in range(0,self.numCPG):
            for currentColumn in range(0,self.numHidden):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow + self.numSensors + self.numHidden , targetNeuronName = currentColumn+self.numSensors , weight = self.cpgWeights[currentRow][currentColumn] )
  
        #MOTORS
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Torso_BackRightLeg")
        self.numNeurons +=1
        self.numMotors +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Torso_BackLeftLeg")
        self.numNeurons +=1
        self.numMotors +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Torso_FrontRightLeg")
        self.numNeurons +=1
        self.numMotors +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "Torso_FrontLeftLeg")
        self.numNeurons +=1
        self.numMotors +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "BackRightLeg_BackRightLower")
        self.numNeurons +=1
        self.numMotors +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "BackLeftLeg_BackLeftLower")
        self.numNeurons +=1
        self.numMotors +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "FrontRightLeg_FrontRightLower")
        self.numNeurons +=1
        self.numMotors +=1
        pyrosim.Send_Motor_Neuron( name = self.numNeurons , jointName = "FrontLeftLeg_FrontLeftLower")
        self.numNeurons +=1
        self.numMotors +=1

        if(not weights):
            self.motorWeights = (np.random.rand(self.numHidden, self.numMotors))*2-1

        for currentRow in range(0,self.numHidden):
            for currentColumn in range(0,self.numMotors):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow + self.numSensors + self.numCPG, targetNeuronName = currentColumn+self.numSensors+self.numHidden +self.numCPG, weight = self.motorWeights[currentRow][currentColumn] )

        



        pyrosim.End()

    def Create_Robot(self, weights=None):
        self.Generate_Brain(weights)
        self.Generate_Body()