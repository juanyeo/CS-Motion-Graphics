import glfw
import numpy as np
import time
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.arrays import vbo
from Particle import Particle
from ParticleSystem import ParticleSystem
from Solver import Solver

class Drawer:
    def __init__(self, scale, interval):
        self.scale = scale
        self.interval = interval
        self.solver = Solver()

    def drawFrame(self):
        glBegin(GL_LINES)
        glColor3ub(80, 80, 80)
        for s in np.arange(-self.scale, self.scale+self.interval, self.interval):
            glVertex3f(s, 0, self.scale)
            glVertex3f(s, 0, -self.scale)
            glVertex3f(self.scale, 0, s)
            glVertex3f(-self.scale, 0, s)
        glEnd()

    def drawBaseVectors(self):
        glBegin(GL_LINES)
        glColor3ub(255, 0, 0)
        glVertex3fv(np.array([0., 0., 0.]))
        glVertex3fv(np.array([0.25, 0., 0.]))
        glColor3ub(0, 255, 0)
        glVertex3fv(np.array([0., 0., 0.]))
        glVertex3fv(np.array([0., 0.25, 0.]))
        glColor3ub(0, 0, 255)
        glVertex3fv(np.array([0., 0., 0.]))
        glVertex3fv(np.array([0., 0., 0.25]))
        glEnd()

    def render(self, angle, height, distance, motion, frame_number):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        
        # manipulate projection
        #glOrtho(-1, 1, -1, 1, 1, 10)
        #glFrustum(-1, 1, -1, 1, 1, 10)
        gluPerspective(90, 1, 1, 1000)

        camera_position = [10.0 * distance * np.sin(angle), height * distance, 10.0 * distance * np.cos(angle)]
        object_position = [0, 20, 80]

        gluLookAt(camera_position[0] + object_position[0],
                  camera_position[1] + object_position[1],
                  camera_position[2] + object_position[2],
                  object_position[0], object_position[1], object_position[2], 0,1,0)
        #gluLookAt(distance*np.sin(angle), distance, distance*np.cos(angle), 0,0,0, 0,1,0)
        #glTranslatef(displacement[0], displacement[1], displacement[2])

        self.drawFrame()
        self.drawBaseVectors()
        glColor3ub(0, 255, 0)
        self.drawMotion(motion, frame_number)
    
    def drawMotion(self, motion, frame_number):
        if frame_number < motion.number_of_frames:
            #glPushMatrix()
            
            posture = motion.posture_list[frame_number]
            glMultMatrixf(posture.root_position.T)
            self.drawBody(posture.skeleton.root, posture)
            #glPopMatrix()

    def drawSinglePosture(self, posture, height):
        #glMultMatrixf(posture.root_position.T)\
        glTranslatef(0.0, height, 0.0)
        self.drawBody(posture.skeleton.root, posture)
        glTranslatef(0.0, -height, 0.0)
            
    def drawBody(self, joint, posture):
        if not joint.is_end:
            glPushMatrix()

            # glMultMatrixf(posture.getAffineMatrix(joint).T)
            glMultMatrixf(posture.getTranslationMatrix(joint).T)
            glMultMatrixf(posture.getRotationMatrix(joint).T)

            for child in joint.children:
                #self.drawBodyPartLine([0, 0, 0], child.offset)
                #self.drawBodyPartCube(child.offset)
                self.drawBodyPartPlane(child.offset)

            for child in joint.children:
                self.drawBody(child, posture)

            glPopMatrix()

    def drawBodyPartLine(self, start_point, end_point):
        glBegin(GL_LINES)
        glVertex3f(start_point[0], start_point[1], start_point[2])
        glVertex3f(end_point[0], end_point[1], end_point[2])
        glEnd()

    def drawBodyPartCube(self, offset):
        #glScaled(size[0], size[1], size[2])

        n = np.array( [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]], dtype=GLfloat)

        faces = np.array([[0, 1, 2, 3], [3, 2, 6, 7], [7, 6, 5, 4],
                        [4, 5, 1, 0], [5, 6, 2, 1], [7, 4, 0, 3]], dtype=GLint)

        #v = np.array([[-.5, -.5, -.5], [-.5, -.5, .5], [-.5, .5, .5],
		#		[-.5, .5, -.5], [.5, -.5, -.5], [.5, -.5, .5],
		#		[.5, .5, .5], [.5, .5, -.5]], dtype=GLfloat)
        v = np.array([[-.5, 0., -.5], [-.5, 0., .5], [-.5 + offset[0], offset[1], .5 + offset[2]],
				[-.5 + offset[0], offset[1], -.5 + offset[2]], [.5, 0., -.5], [.5, 0., .5],
				[.5 + offset[0], offset[1], .5 + offset[2]], [.5 + offset[0], offset[1], -.5 + offset[2]]], dtype=GLfloat)
        
        for i in range(0,6):
            glBegin(GL_QUADS)
            glNormal3fv(n[i])
            glVertex3fv(v[faces[i][0]])
            glVertex3fv(v[faces[i][1]])
            glVertex3fv(v[faces[i][2]])
            glVertex3fv(v[faces[i][3]])
            glEnd()

    def drawBodyPartPlane(self, offset):
        scale = np.linalg.norm(offset) * 0.03
        
        glBegin(GL_QUADS)
        glVertex3fv([-scale, 0., -scale])
        glVertex3fv([scale, 0., scale])
        glVertex3fv([-scale + offset[0], offset[1], -scale + offset[2]])
        glVertex3fv([scale + offset[0], offset[1], scale + offset[2]])
        glEnd()

    def drawLine(self, start_point, end_point):
        glBegin(GL_LINES)
        glVertex3f(start_point[0], start_point[1], start_point[2])
        glVertex3f(end_point[0], end_point[1], end_point[2])
        glEnd()

    def drawPoint(self, point):
        glBegin(GL_POINTS)
        glVertex3f(point[0], point[1], point[2])
        glEnd()

    def drawParticles(self, particleSystem, run, fps):
        delta_t = 0.008
        step = int(1/(fps * delta_t))*10

        if run == True:
            for i in range(step):
                #self.solver.eulerStep(particleSystem, delta_t)
                self.solver.semiImplicitEulerStep(particleSystem, delta_t)

        glLineWidth(2)
        glPointSize(12)
        for particle in particleSystem.particles:
            if (particle.detected):
                glColor3ub(255, 0, 0)
            else:
                glColor3ub(204, 154, 255)
            self.drawPoint(particle.x)

            glColor3ub(127, 0, 254)
            for neighbor in particle.neighbors:
                self.drawLine(particle.x, neighbor.x)
        glLineWidth(1)

    def drawCube(self, height):
        #height = np.sqrt((end_point[0] ** 2) + (end_point[1] ** 2) + (end_point[2] ** 2))
        half = 20.0
        
        glBegin(GL_QUADS) 
        # (4, 5, 6, 7)
        glVertex3f(-half, 0., half) 
        glVertex3f(half, 0., half) 
        glVertex3f(half, 0., -half) 
        glVertex3f(-half, 0., -half)

        # (0, 1, 2, 3)
        glVertex3f(-half, height, half) 
        glVertex3f(half, height, half) 
        glVertex3f(half, height, -half) 
        glVertex3f(-half, height, -half)

        # (0, 3, 4, 7)
        glVertex3f(-half, height, half) 
        glVertex3f(-half, height, -half) 
        glVertex3f(-half, 0., half) 
        glVertex3f(-half, 0., -half)

        # (1, 2, 5, 6)
        glVertex3f(half, height, half) 
        glVertex3f(half, height, -half) 
        glVertex3f(half, 0., half) 
        glVertex3f(half, 0., -half)

        # (2, 3, 6, 7)
        glVertex3f(half, height, -half) 
        glVertex3f(-half, height, -half) 
        glVertex3f(half, 0., -half) 
        glVertex3f(-half, 0., -half)

        # (0, 1, 4, 5)
        glVertex3f(half, height, half) 
        glVertex3f(-half, height, half) 
        glVertex3f(half, 0., half) 
        glVertex3f(-half, 0., half)

        glEnd()

    def calculateAngle(self, point):
        pass