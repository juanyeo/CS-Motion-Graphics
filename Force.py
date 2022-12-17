from abc import *

class Force(metaclass=ABCMeta):

    @abstractclassmethod
    def getForce(self):
        pass

# Gravity, Viscous Drag, Damped Spring, Friction, Eternal