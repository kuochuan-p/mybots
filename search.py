import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER

os.system("rm brain0.nndf brain1.nndf")
os.system("rm fitness0.txt fitness1.txt") 


phc = PARALLEL_HILL_CLIMBER()
phc.Show_Best()

