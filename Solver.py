import numpy as np

class Solver:
    def __init__(self):
        self.G = -9.8
        self.k_viscous = 0.1
        self.k_s = 1000
        self.k_d = 100
        self.P = np.array([0, 0, 0]) # Collision - plane point
        self.N = np.array([0, 1, 0]) # Collision - normal force
        self.threshold = 0.1 # Collision - epsilon
        self.k_r = 0.1 # Collision - response constant
        self.friction_coefficient = 0.42
        self.particle_velocity_threshold = 0.001

    def getGravityForce(self, particle):
        F_Gravity = np.zeros(3)

        F_Gravity[1] = particle.m * self.G

        return F_Gravity

    def getViscousDragForce(self, particle):
        F_Viscous = -self.k_viscous * np.array(particle.v)

        return F_Viscous

    def getSpringForce(self, p1, p2, r):
        d_x = np.array(p1.x) - np.array(p2.x)
        ld_xl = self.getParticleDistance(p1.x, p2.x)
        d_v = np.array(p1.v) - np.array(p2.v)

        F_Spring = -((self.k_s * (ld_xl - r) + self.k_d * (np.dot(d_v, d_x.T) / ld_xl)) * (d_x / ld_xl))
        return F_Spring

    def getFrictionForce(self, particle):
        Fn = np.array([0, -((particle.m * self.G) + particle.f[1]), 0])
        lFnl = np.sqrt(np.sum(Fn**2))
        Vt = np.array([particle.v[0], 0, particle.v[2]])
        lVtl = np.sqrt(np.sum(Vt**2))

        F_Friction = -(self.friction_coefficient * lFnl * (Vt / lVtl))
        return F_Friction

    def calculateForces(self, particleSystem):
        for i in range(len(particleSystem.particles)):
            particle = particleSystem.particles[i]
            F = np.zeros(3)

            # Gravity Force (Y direction)
            F += self.getGravityForce(particle)

            # Viscous Drag Force
            F += self.getViscousDragForce(particle)

            # Damped Spring Force
            for j in particleSystem.connection[i]:
                neighbor = particleSystem.particles[j]
                r = self.getParticleDistance(particleSystem.origin_positions[i], particleSystem.origin_positions[j])
                F += self.getSpringForce(particle, neighbor, r)

            # apply Force
            particle.f = F

    def getParticleDerivative(self, particleSystem):
        self.calculateForces(particleSystem)

        derivative = []
        for particle in particleSystem.particles:
            derivative.extend(particle.v)
            a = np.array(particle.f)/particle.m
            derivative.extend(list(a))
        
        return derivative

    def collisionDetection(self, particleSystem, collision):
        for i in range(len(particleSystem.particles)):
            particle = particleSystem.particles[i]

            xdotn = np.dot(particle.x - self.P, self.N.T)
            ndotv = np.dot(self.N, particle.v.T)
            if (xdotn < self.threshold) and (ndotv < 0):
                collision.append(i)
                particle.detected = True
            else:
                particle.detected = False

    def collisionResponse(self, particleSystem, collision, delta_t):
        if len(collision) == 0:
            return
        for i in collision:
            particle = particleSystem.particles[i]

            F_Friction = self.getFrictionForce(particle)
            particle.v += (F_Friction / particle.m) * delta_t

            # Collision Response
            Vn = np.dot(self.N, particle.v.T) * self.N
            Vt = particle.v - Vn

            particle.v = Vt - self.k_r * Vn

    def eulerStep(self, particleSystem, delta_t):
        derivative = np.array(self.getParticleDerivative(particleSystem))
        current = np.array(particleSystem.getState())
        particleSystem.setState(current + derivative * delta_t)
        
        collision = []
        
        self.collisionDetection(particleSystem, collision)
        self.collisionResponse(particleSystem, collision, delta_t)

        for i in range(len(particleSystem.particles)):
            particle = particleSystem.particles[i]
            if (np.sum(particle.v**2) < self.particle_velocity_threshold):
                particle.v = [0, 0, 0]

        particleSystem.time = particleSystem.time + delta_t

    def semiImplicitEulerStep(self, particleSystem, delta_t):
        derivative = np.array(self.getParticleDerivative(particleSystem))

        for i in range(len(particleSystem.particles)):
            derivative[i*6 : i*6+3] += delta_t * derivative[i*6+3 : i*6+6]

        current = np.array(particleSystem.getState())
        particleSystem.setState(current + derivative*delta_t)
        
        collision = []
        self.collisionDetection(particleSystem, collision)
        self.collisionResponse(particleSystem, collision, delta_t)

        for i in range(len(particleSystem.particles)):
            particle = particleSystem.particles[i]
            if (np.sum(particle.v**2) < self.particle_velocity_threshold):
                particle.v = [0, 0, 0]

        particleSystem.time = particleSystem.time + delta_t

    def getParticleDistance(self, position_1, porition_2):
        diff = np.array(position_1) - np.array(porition_2)

        return np.sqrt(np.sum(diff**2))


            
        