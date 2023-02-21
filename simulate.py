from simulation import SIMULATION

# import pybullet as p
# import pybullet_data
# import time as t
# import pyrosim.pyrosim as pyrosim
# import numpy
# import random
# import constants as c

simulation = SIMULATION()
simulation.Run()


# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.setGravity(0,0,c.GRAVITY)

# planeId = p.loadURDF("plane.urdf")
# robotId = p.loadURDF("body.urdf")

# p.loadSDF("world.sdf")

# pyrosim.Prepare_To_Simulate(robotId)

# #numpy vectors
# backLegSensorValues = numpy.zeros(1000)
# frontLegSensorValues = numpy.zeros(1000)

# #get angle values
# x = numpy.linspace(0,2*c.PI, 1000)
# backLegtargetAngles = numpy.sin(x*c.backLegFrequency + c.backLegPhaseOffset)*c.backLegAmplitude
# frontLegtargetAngles = numpy.sin(x*c.frontLegFrequency + c.frontLegPhaseOffset)*c.frontLegAmplitude

# #save angle values
# # numpy.save("data/backLegTargetAngles.npy", backLegtargetAngles)
# # numpy.save("data/frontLegTargetAngles.npy", frontLegtargetAngles)

# for i in range(0,1000):
#     p.stepSimulation()

#     #get sensor values
#     backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#     frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

#     pyrosim.Set_Motor_For_Joint(
#     bodyIndex = robotId,
#     jointName = "Torso_BackLeg",
#     controlMode = p.POSITION_CONTROL,
#     targetPosition = backLegtargetAngles[i],
#     maxForce = 500)

#     pyrosim.Set_Motor_For_Joint(
#     bodyIndex = robotId,
#     jointName = "Torso_FrontLeg",
#     controlMode = p.POSITION_CONTROL,
#     targetPosition = frontLegtargetAngles[i],
#     maxForce = 50)

#     t.sleep(1/100)

# p.disconnect()

# #save data
# numpy.save("data/backLegSensorValues.npy", backLegSensorValues)
# numpy.save("data/frontLegSensorValues.npy", frontLegSensorValues)

