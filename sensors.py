

import numpy as np
from numpy.random import normal



class PositonSensor:
    def __init__(self, dimension=2):
        self.zk = np.ones(shape=(dimension, 1))
        self.step = 1.0 #meter

    def get(self):
        #move target
        zk = self.zk +  self.step

        #add noise
        zk[0][0] += normal(loc=0.0, scale=0.5)
        zk[1][0] += normal(loc=0.0, scale=0.5)

        self.zk = zk

        return self.zk


