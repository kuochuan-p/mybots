import pybullet as p
import pybullet_data
import constants as c

class WORLD:
    def __init__(self):
        planeId = p.loadURDF("plane.urdf")
        p.loadSDF("world.sdf")