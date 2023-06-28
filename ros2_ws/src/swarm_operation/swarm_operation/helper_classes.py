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

class RollingAverage():
    def __init__(self, size):
        self._size = size
        self._data = []
        self._average = 0

    def add(self, value):
        self._data.append(value)
        if len(self._data) > self._size:
            self._data.pop(0)
    
    def get(self):
        return sum(self._data)/len(self._data)