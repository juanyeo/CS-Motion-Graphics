import numpy as np

class Joint:
    def __init__(self, name):
        self.name = name
        self.parent = None
        self.offset = np.zeros(3)
        self.channels = []
        self.children = []
        self.is_end = False
    
    def addChild(self, child):
        self.children.append(child)