import numpy as np
from MatrixUtil import *

class ExternalForce:
    def __init__(self, m, x1, x2):
        self.weight = m
        self.edge_x1 = np.array(x1)
        self.edge_x2 = np.array(x2)
        self.delta_t = 0.008
        self.momentum = self.estimateMomentum()
        self.mid_distance = 1.
        self.epsilon = 0.01

    def detectContact(self, particleSystem):
        midpoint = np.array([0., 0., 0.])
        count = 0.

        for i in range(len(particleSystem.particles)):
            midpoint += particleSystem.particles[i].x
            count += 1
        midpoint /= count

        self.mid_distance = getParticleDistance(self.edge_x2, midpoint)

        return self.mid_distance < 1.5 * getParticleDistance(particleSystem.particles[0].x, midpoint)

    def applyForce(self, particleSystem):
        F = self.momentum / self.delta_t

        for i in range(len(particleSystem.particles)):
            particle = particleSystem.particles[i]

            distance = getParticleDistance(self.edge_x2, particle.x)
            closeness = (self.mid_distance - distance) / self.mid_distance

            if (closeness < 0):
                continue

            particle.f += self.epsilon * closeness * F

    def estimateMomentum(self):
        v = (self.edge_x2 - self.edge_x1) / self.delta_t
        
        return self.weight * v