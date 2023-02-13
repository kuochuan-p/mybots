import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load("data/backLegSensorValues.npy")
frontLegSensorValues = numpy.load("data/frontLegSensorValues.npy")

backLegPlot = matplotlib.pyplot.plot(backLegSensorValues, label="back leg", linewidth=4)
frontLegPlot = matplotlib.pyplot.plot(frontLegSensorValues, label="front leg", linewidth=2)
matplotlib.pyplot.legend()

matplotlib.pyplot.show()    