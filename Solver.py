import numpy as np
from MatrixUtil import *
from Particle import Particle
from GravityForce import GravityForce
from ViscousForce import ViscousForce
from SpringForce import SpringForce
from FrictionForce import FrictionForce

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

    def calculateForces(self, particleSystem):
        for i in range(len(particleSystem.particles)):
            particle = particleSystem.particles[i]
            F = np.zeros(3)

            # Gravity Force (Y direction)
            gravity = GravityForce(self.G)
            F += gravity.getForce(particle)

            # Viscous Drag Force
            viscous = ViscousForce(self.k_viscous)
            F += viscous.getForce(particle)

            # Damped Spring Force
            spring = SpringForce(self.k_s, self.k_d)
            for j in particleSystem.connection[i]:
                neighbor = particleSystem.particles[j]
                r = getParticleDistance(particleSystem.origin_positions[i], particleSystem.origin_positions[j])
                F += spring.getForce(particle, neighbor, r)

            # Sticked Spring Force
            if (particleSystem.is_sticked):
                stick = Particle()
                if (i == 0):
                    stick.x = particleSystem.stick_point1
                    F += spring.getForce(particle, stick, 9.)
                elif (i == 1):
                    stick.x = particleSystem.stick_point1
                    F += spring.getForce(particle, stick, 2.)
                elif (i == 3):
                    stick.x = particleSystem.stick_point2
                    F += spring.getForce(particle, stick, 2.)
                elif (i == 4):
                    stick.x = particleSystem.stick_point2
                    F += spring.getForce(particle, stick, 9.)
                F += [0, 0, -200]

            # apply Force
            particle.f = F

    def getParticleDerivative(self, particleSystem, externalForce):
        self.calculateForces(particleSystem)

        if (externalForce != None and externalForce.detectContact(particleSystem)):
            externalForce.applyForce(particleSystem)
            #print("공이 충돌했습니다!!")

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
        friction = FrictionForce(self.G, self.friction_coefficient)

        for i in collision:
            particle = particleSystem.particles[i]

            F_Friction = friction.getForce(particle)
            particle.v += (F_Friction / particle.m) * delta_t

            # Collision Response
            Vn = np.dot(self.N, particle.v.T) * self.N
            Vt = particle.v - Vn

            particle.v = Vt - self.k_r * Vn

    def eulerStep(self, particleSystem, delta_t, externalForce):
        derivative = np.array(self.getParticleDerivative(particleSystem, externalForce))
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

    def semiImplicitEulerStep(self, particleSystem, delta_t, externalForce):
        derivative = np.array(self.getParticleDerivative(particleSystem, externalForce))

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


            
        