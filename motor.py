import constants as c
import numpy
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.backLegAmplitude
        self.frequency = c.backLegFrequency
        
        if(self.jointName == "Torso_BackLeg"):
            self.frequency = c.backLegFrequency*2
        self.offset = c.backLegPhaseOffset

        x = numpy.linspace(0,2*c.PI, 1000)
        self.motorValues = numpy.sin(x*self.frequency + self.offset)*self.amplitude
    def Set_Value(self, robotId, i):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = self.motorValues[i],
            maxForce = 50)

    def Save_Values(self):
        numpy.save("data/"+self.jointName+".npy", self.motorValues)
