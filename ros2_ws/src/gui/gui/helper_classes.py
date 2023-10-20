"""
Helper classes used in the GUI creation
"""

class RollingAverage():
    """ Class that allows a rolling average to be created from a list """
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
