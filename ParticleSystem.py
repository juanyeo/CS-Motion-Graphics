
class ParticleSystem:
    def __init__(self):
        self.particles = []
        self.n = 0
        self.time = 0
        self.connection = []
        self.origin_positions = []

    def getState(self):
        state = []

        for particle in self.particles:
            state.extend(particle.x)
            state.extend(particle.v)

        return state

    def setState(self, source):
        i = 0

        for particle in self.particles:
            particle.x = source[i : i + 3]
            particle.v = source[i + 3 : i + 6]
            i += 6
        
    def getPositions(self):
        positions = []

        for particle in self.particles:
            positions.append(particle.x)
        
        return positions

    def initNeighborParticles(self):
        for i in range(len(self.particles)):
            particle = self.particles[i]
            neighbors = self.connection[i]

            for j in neighbors:
                particle.neighbors.append(self.particles[j])
