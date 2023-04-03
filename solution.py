import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import copy
import time
import constants as c


class SOLUTION:

    def __init__(self, ident) -> None:
        self.myID = ident
        self.weights = (np.random.rand(c.numSensorNeurons, c.numMotorNeurons))*2-1
    
    def Set_ID(self, ident):
        self.myID = ident

    def Evaluate(self, dOrG):
        self.Create_Robot()
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
        randRow = random.randint(0,c.numSensorNeurons-1)
        randCol = random.randint(0,c.numMotorNeurons-1)

        self.weights[randRow][randCol] = random.random()*2-1


    def Create_world(self):

        pyrosim.Start_SDF("world.sdf")
        length, width, height = 1,1,1
        x,y,z = 0,0,0.5
        pyrosim.End()

    def Generate_Body(self):

        pyrosim.Start_URDF("body.urdf")
        length, width, height = 3,1,1
        x,y,z = 0,0,2.5
        
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])

        #back legs
        pyrosim.Send_Joint( name = "Torso_BackRightLeg" , parent= "Torso" , child = "BackRightLeg" , type = "revolute", position = [length/2,width/2,z-.5], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackRightLeg", pos=[0,0,-.5] , size=[.2,0.2,1])
        pyrosim.Send_Joint( name = "Torso_BackLeftLeg" , parent= "Torso" , child = "BackLeftLeg" , type = "revolute", position = [length/2,-width/2,z-.5], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackLeftLeg", pos=[0,0,-.5] , size=[.2,0.2,1])

        pyrosim.Send_Joint( name = "BackRightLeg_BackRightLower" , parent= "BackRightLeg" , child = "BackRightLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackRightLower", pos=[0,0,-.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint( name = "BackLeftLeg_BackLeftLower" , parent= "BackLeftLeg" , child = "BackLeftLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="BackLeftLower", pos=[0,0,-.5] , size=[0.2,0.2,1])

        #front legs
        pyrosim.Send_Joint( name = "Torso_FrontRightLeg" , parent= "Torso" , child = "FrontRightLeg" , type = "revolute", position = [-length/2,.5,z-.5], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontRightLeg", pos=[0,0,-.5] , size=[.2,0.2,1])
        pyrosim.Send_Joint( name = "Torso_FrontLeftLeg" , parent= "Torso" , child = "FrontLeftLeg" , type = "revolute", position = [-length/2,-.5,z-.5], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontLeftLeg", pos=[0,0,-.5] , size=[.2,0.2,1])

        pyrosim.Send_Joint( name = "FrontRightLeg_FrontRightLower" , parent= "FrontRightLeg" , child = "FrontRightLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontRightLower", pos=[0,0,-.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint( name = "FrontLeftLeg_FrontLeftLower" , parent= "FrontLeftLeg" , child = "FrontLeftLower" , type = "revolute", position = [0,0,-1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="FrontLeftLower", pos=[0,0,-.5] , size=[0.2,0.2,1])

        

        pyrosim.End()

    def Generate_Brain(self):

        pyrosim.Start_URDF("brain"+str(self.myID)+".nndf")

        neuronName = 0

        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "BackRightLower")
        neuronName +=1
        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "BackLeftLower")
        neuronName +=1
        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "FrontRightLower")
        neuronName +=1
        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "FrontLeftLower")
        neuronName +=1

        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_BackRightLeg")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_BackLeftLeg")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_FrontRightLeg")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_FrontLeftLeg")
        neuronName +=1

        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "BackRightLeg_BackRightLower")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "BackLeftLeg_BackLeftLower")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "FrontRightLeg_FrontRightLower")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "FrontLeftLeg_FrontLeftLower")
        neuronName +=1
        
        for currentRow in range(0,c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+c.numSensorNeurons , weight = self.weights[currentRow][currentColumn] )

        pyrosim.End()

    def Create_Robot(self):
        self.Generate_Brain()
        self.Generate_Body()