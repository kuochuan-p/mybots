import pyrosim.pyrosim as pyrosim
import random

def Create_world():

    pyrosim.Start_SDF("world.sdf")
    length, width, height = 1,1,1
    x,y,z = 0,0,0.5
    pyrosim.End()

def Create_Robot():
    Generate_Brain()
    Generate_Body()


def Generate_Body():

    pyrosim.Start_URDF("body.urdf")
    length, width, height = 1,1,1
    x,y,z = 0,0,1.5
    pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [x-0.5,y,z-0.5])
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length,width,height])
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [x+0.5,y,z-0.5])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0+.5,0,-.5] , size=[length,width,height])

    pyrosim.End()

def Generate_Brain():

    pyrosim.Start_URDF("brain.nndf")
    
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

    for i in range(0,3):
        for j in range(3,5):
            pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = random.uniform(-1,1) )






    pyrosim.End()


Create_Robot()
Create_world()