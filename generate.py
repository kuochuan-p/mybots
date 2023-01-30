import pyrosim.pyrosim as pyrosim

length, width, height = 1,1,1
x,y,z = 0,0,0.5
pyrosim.Start_SDF("boxes.sdf")
for i in range(0,5):
    for j in range(0,5):
        for k in range(0,10):
            pyrosim.Send_Cube(name="Box"+str(i)+str(j)+str(k), pos=[x,y,z] , size=[length,width,height])
            z+=1
            length, width, height = .9*length, .9*width, .9*height
        z=.5
        length, width, height = 1,1,1
        x+=1
    x=0
    y+=1

pyrosim.End()
