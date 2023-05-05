import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
from matplotlib import pyplot
import numpy

os.system("rm fitnessValuesConnected.npy fitnessValuesDisconnected.npy")



phc1 = PARALLEL_HILL_CLIMBER(False)
phc1.SaveFitnessValues()

phc2 = PARALLEL_HILL_CLIMBER(True)
phc2.SaveFitnessValues()

connectValues = numpy.load("fitnessValuesConnected.npy")
disconnectValues = numpy.load("fitnessValuesDisconnected.npy")


pyplot.plot(connectValues[0], label="Fully Connected NN")
pyplot.plot(disconnectValues[0], label="Partially Disconnected NN")
pyplot.legend()
pyplot.show()
