import pyrosim.pyrosim as pyrosim

def Create_world():

    pyrosim.Start_SDF("world.sdf")
    length, width, height = 1,1,1
    x,y,z = 0,0,0.5
    pyrosim.End()

def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    length, width, height = 1,1,1
    x,y,z = 0,0,1.5
    pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [x-0.5,y,z-0.5])
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length,width,height])
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [x+0.5,y,z-0.5])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0+.5,0,-.5] , size=[length,width,height])



    pyrosim.End()


Create_Robot()
Create_world()