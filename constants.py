import pybullet as p
import pybullet_data
import time as t
import pyrosim.pyrosim as pyrosim
import numpy
import random

PI = 3.14159265
GRAVITY = -9.8
backLegAmplitude, backLegFrequency, backLegPhaseOffset = PI/4, 5, 0
numberOfGenerations = 10
populationSize = 10

numSensorNeurons = 4
numMotorNeurons = 8

motorJointRange = .5