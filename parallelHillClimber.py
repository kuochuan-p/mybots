from solution import SOLUTION
import constants as c
import copy
import numpy as np

class PARALLEL_HILL_CLIMBER:

    def __init__(self, variant):
        self.variant = variant
        self.nextAvailableID = 0
        self.parents = {}
        self.fitnessValues = np.zeros((c.populationSize, c.numberOfGenerations))

        for i in range(0,c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID, variant)
            self.nextAvailableID += 1
        self.Evolve()

    def Evolve(self):
        
        self.Evaluate(self.parents)
        for currentGeneration in range (0, c.numberOfGenerations):
            self.Evolve_For_One_Generation(currentGeneration)

    def Evolve_For_One_Generation(self, currentGeneration):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Select(currentGeneration)

    def Spawn(self):
        self.children = {}
        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
    
    def Mutate(self):
        for key, value in self.children.items():
            value.Mutate()

    def Select(self, currentGeneration):
        for i in range(0,c.populationSize):
            if(self.parents[i].fitness <= self.children[i].fitness):
                self.parents[i] = self.children[i]
            self.fitnessValues[i, currentGeneration] = self.parents[i].fitness

    def Evaluate(self, solutions):
        for i in range(0,c.populationSize):
            solutions[i].Start_Simulation("DIRECT") 
        
        for i in range(0,c.populationSize):
            solutions[i].Wait_For_Simulation_To_End()
    def Print(self):
        for i in range(0,c.populationSize):
            print("\n\n",self.parents[i].fitness ,"----------",self.children[i].fitness)

    def SaveFitnessValues(self):
        np.save("fitnessValues"+("Connected", "Disconnected")[self.variant]+".npy", self.fitnessValues)

    def Show_Best(self):
        max = -1
        for i in range(0,c.populationSize):
            if self.parents[i].fitness > max:
                max = self.parents[i].fitness
                opt = self.parents[i]
        
        opt.Start_Simulation("GUI")

