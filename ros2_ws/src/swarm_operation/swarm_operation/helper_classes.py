import numpy as np

class RollingList():
    def __init__(self, size, init_values=None):
        self._size = size
        self.data = np.array([init_values]*size)
    
    def append(self, value):
        self.data = np.roll(self.data, 1, axis=0)
        self.data[0] = value
    
    def __getitem__(self, index):
        return self.data[index]