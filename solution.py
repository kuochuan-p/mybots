import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import copy
import time


class SOLUTION:

    def __init__(self, ident) -> None:
        self.myID = ident
        self.weights = (np.random.rand(3, 2))*2-1
    
    def Set_ID(self, ident):
        self.myID = ident

    def Evaluate(self, dOrG):
        self.Create_Robot()
        os.system("python3 simulate.py " + dOrG + " " + str(self.myID) + " &")

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
        randRow = random.randint(0,2)
        randCol = random.randint(0,1)

        self.weights[randRow][randCol] = random.random()*2-1


    def Create_world(self):

        pyrosim.Start_SDF("world.sdf")
        length, width, height = 1,1,1
        x,y,z = 0,0,0.5
        pyrosim.End()

    def Generate_Body(self):

        pyrosim.Start_URDF("body.urdf")
        length, width, height = 1,1,1
        x,y,z = 0,0,1.5
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [x-0.5,y,z-0.5])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length,width,height])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [x+0.5,y,z-0.5])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0+.5,0,-.5] , size=[length,width,height])

        pyrosim.End()

    def Generate_Brain(self):

        pyrosim.Start_URDF("brain"+str(self.myID)+".nndf")
        
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        for currentRow in range(0,3):
            for currentColumn in range(0,2):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+3 , weight = self.weights[currentRow][currentColumn] )

        pyrosim.End()

    def Create_Robot(self):
        self.Generate_Brain()
        self.Generate_Body()