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
        length, width, height = 1,1,1
        x,y,z = 0,0,1
        
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])
        
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [x,y+.5,z], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,.5,0] , size=[0.2,1,0.2])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [x,y-.5,z], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-.5,0] , size=[0.2,1,0.2])
        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [x-.5,y,z], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-.5,0,0] , size=[1,0.2,0.2])
        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [x+.5,y,z], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[.5,0,0] , size=[1,0.2,0.2])

        pyrosim.Send_Joint( name = "RightLeg_LowerRight" , parent= "RightLeg" , child = "LowerRight" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LowerRight", pos=[0,0,-.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint( name = "LeftLeg_LowerLeft" , parent= "LeftLeg" , child = "LowerLeft" , type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LowerLeft", pos=[0,0,-.5] , size=[0.2,0.2,1])

        pyrosim.Send_Joint( name = "FrontLeg_LowerFront" , parent= "FrontLeg" , child = "LowerFront" , type = "revolute", position = [0,1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LowerFront", pos=[0,0,-.5] , size=[0.2,0.2,1])
        pyrosim.Send_Joint( name = "BackLeg_LowerBack" , parent= "BackLeg" , child = "LowerBack" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LowerBack", pos=[0,0,-.5] , size=[0.2,0.2,1])

        pyrosim.End()

    def Generate_Brain(self):

        pyrosim.Start_URDF("brain"+str(self.myID)+".nndf")

        neuronName = 0
        
        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "LowerFront")
        neuronName +=1
        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "LowerBack")
        neuronName +=1
        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "LowerRight")
        neuronName +=1
        pyrosim.Send_Sensor_Neuron(name = neuronName , linkName = "LowerBack")
        neuronName +=1

        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_BackLeg")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_FrontLeg")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_LeftLeg")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "Torso_RightLeg")
        neuronName +=1

        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "FrontLeg_LowerFront")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "BackLeg_LowerBack")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "RightLeg_LowerRight")
        neuronName +=1
        pyrosim.Send_Motor_Neuron( name = neuronName , jointName = "LeftLeg_LowerLeft")
        neuronName +=1

        for currentRow in range(0,c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+c.numSensorNeurons , weight = self.weights[currentRow][currentColumn] )

        pyrosim.End()

    def Create_Robot(self):
        self.Generate_Brain()
        self.Generate_Body()