import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load("data/backLegSensorValues.npy")
matplotlib.pyplot.plot(backLegSensorValues)
matplotlib.pyplot.show()