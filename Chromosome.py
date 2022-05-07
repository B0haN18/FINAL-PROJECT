
import math
import random
import pprint
import pickle
import sys
import os
from datetime import datetime
import util

class Chromosome:
    def __init__(self, bounds, NPC_size, time_size):
        self.y = 0
        self.scenario = [[[] for i in range(time_size)] for j in range(NPC_size)] # This scenario
        self.bounds = bounds
        self.code_x1_length = NPC_size 
        self.code_x2_length = time_size
        self.timeoutTime = 300 # in seconds, timeout timer for simulator execution per each scenario simulation

    def fix_init(self):
        for i in range(self.code_x1_length):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                v = (self.bounds[0][0] + self.bounds[0][1]) / float(2) #random.uniform(self.bounds[0][0], self.bounds[0][1])        # Init velocity
                a = 3  # Keep straight #random.randrange(self.bounds[1][0], self.bounds[1][1])      # Init action
                self.scenario[i][j].append(v)
                self.scenario[i][j].append(a)

    def rand_init(self):
        for i in range(self.code_x1_length):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                v = random.uniform(self.bounds[0][0], self.bounds[0][1])        # Init velocity
                a = random.randrange(self.bounds[1][0], self.bounds[1][1])      # Init action
                self.scenario[i][j].append(v)
                self.scenario[i][j].append(a)

# Test
if __name__ == '__main__':
    a = [[10, 30], [0, 2]]
    chromosome = Chromosome(a, 2, 10)
    print(chromosome.code_x1_length)
    #pprint.pprint(chromosome.scene)

