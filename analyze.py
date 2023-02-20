import numpy
import matplotlib.pyplot

btargetAngles = numpy.load("data/backLegTargetAngles.npy")
ftargetAngles = numpy.load("data/frontLegTargetAngles.npy")

# backLegSensorValues = numpy.load("data/backLegSensorValues.npy")
# frontLegSensorValues = numpy.load("data/frontLegSensorValues.npy")

# backLegPlot = matplotlib.pyplot.plot(backLegSensorValues, label="back leg", linewidth=4)
# frontLegPlot = matplotlib.pyplot.plot(frontLegSensorValues, label="front leg", linewidth=2)

banglePlot = matplotlib.pyplot.plot(btargetAngles)
fanglePlot = matplotlib.pyplot.plot(ftargetAngles)

matplotlib.pyplot.legend()

matplotlib.pyplot.show()    