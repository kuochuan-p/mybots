import pyrosim.pyrosim as pyrosim

def Create_world():

    pyrosim.Start_SDF("world.sdf")
    length, width, height = 1,1,1
    x,y,z = 0,0,0.5
    pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length,width,height])
    pyrosim.End()

def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    length, width, height = 1,1,1
    x,y,z = 0,0,0.5
    pyrosim.Send_Cube(name="Link0", pos=[x,y,z] , size=[length,width,height])
    pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [x+.5,y,z+.5])
    pyrosim.Send_Cube(name="Link1", pos=[.5,0,.5] , size=[length,width,height])


    pyrosim.End()


Create_Robot()
Create_world()