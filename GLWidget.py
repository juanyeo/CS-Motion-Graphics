from PyQt5.QtWidgets import QApplication, QOpenGLWidget
from PyQt5 import QtGui, QtCore
from OpenGL.GL import *
from OpenGL.GLU import *
import glfw
from matplotlib.backend_bases import MouseButton, MouseEvent
from matplotlib.widgets import Button
import numpy as np
import sys
from BvhParser import BvhParser
from Joint import Joint
from Motion import Motion
from Posture import Posture
from Drawer import *
import MatrixUtil as util
import MotionEditor as editor
from Particle import Particle
from ParticleSystem import ParticleSystem
from Solver import Solver
from ExternalForce import ExternalForce

file_path = './motion_files/run.bvh'

gCamAngle = np.radians(-55)
gCamHeight = 12
gCamDistance = 15.0
gCamDeltaX = 30.
gCamDeltaZ = 10.
ik_joints = [["rShldr", "rForeArm", "rHand"], ["lShldr", "lForeArm", "lHand"]
            , ["rThigh", "rShin", "rFoot"], ["lThigh", "lShin", "lFoot"]]

class GLWidget(QOpenGLWidget):
    def __init__(self, parent = None):
        super(GLWidget, self).__init__(parent)
        self.motion = BvhParser(file_path).getMotion()
        
        self.warp_motion = None
        self.warp_frame = 0
        self.warp_interval = 50
        self.warp_run = False
        self.warp_type = "all"
        ik_root_position = np.zeros(len(self.motion.posture_list[0].data))
        #ik_root_position[:3] = self.motion.posture_list[0].data[:3]
        self.ik_posture = Posture(self.motion.skeleton, ik_root_position)
        #self.ik_edge_point = [0., 0., 0.]

        self.current_frame = 0
        self.number_of_frames = self.motion.number_of_frames
        self.fps = self.motion.fps

        self.run = False
        self.mouse_x = 0.
        self.mouse_y = 0.
        self.mode_ik = 0

        self.cube_system = self.makeParticleCube()
        self.foot_position = [100, 0, 100]

        self.cloak_system = None
        
        self.setMouseTracking(False)
        self.setAcceptDrops(True)

    def initializeMotionWarp(self):
        self.run = False
        self.current_frame = 0
        if self.warp_run:
            self.warp_run = False
        else:
            self.warp_run = True
            self.warp_motion = editor.makeMotionWarping(self.motion, self.ik_posture, self.warp_frame, 
                            max(2, self.warp_frame - self.warp_interval), min(self.number_of_frames-2, self.warp_frame + self.warp_interval), self.warp_type)

        self.repaint()

    def updateMotionWarp(self):
        self.current_frame = 0
        self.warp_motion = editor.makeMotionWarping(self.motion, self.ik_posture, self.warp_frame, 
                            max(2, self.warp_frame - self.warp_interval), min(self.number_of_frames-2, self.warp_frame + self.warp_interval), self.warp_type)
        self.repaint()
    
    def initializeGL(self):
        #glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(90, 1, 1, 1000)
    
    def paintGL(self):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        camera_position = [10.0 * gCamDistance * np.sin(gCamAngle), 
                           gCamHeight * gCamDistance, 10.0 * gCamDistance * np.cos(gCamAngle)]
        object_position = [gCamDeltaX, 50, gCamDeltaZ]

        gluLookAt(camera_position[0] + object_position[0],
                  camera_position[1] + object_position[1],
                  camera_position[2] + object_position[2],
                  object_position[0], object_position[1], object_position[2], 0,1,0)

        drawer = Drawer(800, 40)
        drawer.drawFrame()

        #ik_root_position = np.zeros(len(self.motion.posture_list[0].data))
        current_foot_position = self.foot_position
        if self.current_frame < self.motion.number_of_frames:
            current_foot_position = self.motion.posture_list[self.current_frame].getJointPosition("rHand") # rHand rFoot
        external = ExternalForce(10, self.foot_position, current_foot_position)
        drawer.drawParticles(self.cube_system, self.run, self.fps, external, 12, (204, 154, 255), (127, 0, 254))
        self.foot_position = current_foot_position

        if self.current_frame == 0:
            self.cloak_system = self.makeParticleCloak(
            self.motion.posture_list[0].getJointPosition("rShldr"),
            self.motion.posture_list[0].getJointPosition("lShldr"))
        if self.current_frame < self.motion.number_of_frames:
            self.cloak_system.stick_point1 = self.motion.posture_list[self.current_frame].getJointPosition("rShldr")
            self.cloak_system.stick_point2 = self.motion.posture_list[self.current_frame].getJointPosition("lShldr")
        drawer.drawParticles(self.cloak_system, self.run, self.fps, None, 3, (255, 153, 51), (255, 51, 51))

        height = self.motion.posture_list[0].data[1]
        if (self.mode_ik > 0):
            glColor3ub(255, 160, 0)
            drawer.drawSinglePosture(self.ik_posture, height)
        #glColor3ub(255, 0, 0)
        #drawer.drawCube(180)
        glColor3ub(0, 255, 0)
        if self.warp_run:
            drawer.drawMotion(self.warp_motion, self.current_frame)
        else:
            drawer.drawMotion(self.motion, self.current_frame)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        global gCamAngle, gCamDeltaX, gCamDeltaZ
        self.run = False
        
        if event.buttons() == QtCore.Qt.LeftButton:
            gCamAngle += np.radians(0.1 * (event.x() - self.mouse_x))

        elif event.buttons() == QtCore.Qt.RightButton:
            gCamDeltaX -= 0.5 * (event.x() - self.mouse_x)
            gCamDeltaZ -= 0.5 * (event.y() - self.mouse_y)
        
        self.mouse_x = event.x()
        self.mouse_y = event.y()
        self.repaint()

    def wheelEvent(self, event: QtGui.QWheelEvent):
        global gCamDistance
        self.run = False

        gCamDistance += 0.01 * event.angleDelta().y()
        self.repaint()

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        self.mouse_x = event.x()
        self.mouse_y = event.y()

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        global gCamDeltaX, gCamDeltaZ, gCamHeight
        self.run = False
        
        if event.key() == QtCore.Qt.Key.Key_W:
            gCamDeltaX += 10.
            #self.calculateIKVectors(0., 0., 1.)
        elif event.key() == QtCore.Qt.Key.Key_S:
            gCamDeltaX -= 10.
        elif event.key() == QtCore.Qt.Key.Key_A:
            gCamDeltaZ += 10.
        elif event.key() == QtCore.Qt.Key.Key_D:
            gCamDeltaZ -= 10.
        elif event.key() == QtCore.Qt.Key.Key_Q:
            gCamHeight += 1
        elif event.key() == QtCore.Qt.Key.Key_E:
            gCamHeight -= 1
        elif event.key() == QtCore.Qt.Key.Key_Z:
            self.moveObject(0, 0, 10)
        elif event.key() == QtCore.Qt.Key.Key_X:
            self.moveObject(10, 0, 0)
        elif event.key() == QtCore.Qt.Key.Key_C:
            self.moveObject(0, 0, -10)
        elif event.key() == QtCore.Qt.Key.Key_V:
            self.moveObject(-10, 0, 0)
        elif event.key() == QtCore.Qt.Key.Key_B:
            self.moveObject(0, 10, 0)
        elif event.key() == QtCore.Qt.Key.Key_N:
            self.moveObject(0, -10, 0)
        elif event.key() == QtCore.Qt.Key.Key_R:
            self.cube_system = self.makeParticleCube()
        self.repaint()

    def dragEnterEvent(self, event: QtGui.QDragEnterEvent):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()
    
    def dropEvent(self, event: QtGui.QDropEvent):
        files = [unicode(u.toLocalFile()) for u in event.mimeData().urls()]
        if files:
            print(files[0])
            self.importNewMotion(files[0])

    def importNewMotion(self, file_path):
        self.run = False
        self.motion = BvhParser(file_path).getMotion()
        self.current_frame = 0
        self.number_of_frames = self.motion.number_of_frames
        self.fps = self.motion.fps

        self.repaint()

    def moveObject(self, x, y, z):
        delta_x = np.array([x, y, z])

        for particle in self.cube_system.particles:
            particle.x += delta_x

    def calculateIKVectors(self, target_point):
        a = self.ik_posture.getJointPosition(ik_joints[self.mode_ik-1][0])
        b = self.ik_posture.getJointPosition(ik_joints[self.mode_ik-1][1])
        c = self.ik_posture.getJointPosition(ik_joints[self.mode_ik-1][2])

        a_gr = self.ik_posture.getGlobalRotationMatrix(ik_joints[self.mode_ik-1][0])
        b_gr = self.ik_posture.getGlobalRotationMatrix(ik_joints[self.mode_ik-1][1])

        a_lr = self.ik_posture.rotation_matrix_dict[ik_joints[self.mode_ik-1][0]]
        b_lr = self.ik_posture.rotation_matrix_dict[ik_joints[self.mode_ik-1][1]]

        new_a_lr, new_b_lr = util.calculateInverseKinematics3(a, b, c, target_point, a_gr, b_gr, a_lr, b_lr)

        self.ik_posture.rotation_matrix_dict[ik_joints[self.mode_ik-1][0]] = new_a_lr
        self.ik_posture.rotation_matrix_dict[ik_joints[self.mode_ik-1][1]] = new_b_lr
        self.repaint()

    def makeParticleCube(self):
        #offset = np.array([40, 60, 0]) # 40, 60, 0
        offset = np.array([40, 360, 0])

        positions = [[15., 6., 0.], [15., 27., 21.], [-15., 21., 21.],
                        [-15., 0., 0.], [15., 27., -21.], [15., 48., 0.],
                        [-15., 42., 0.], [-15., 21., -21.]]
        mass = 10

        cubeSystem = ParticleSystem()

        for i in range(8):
            particle = Particle()
            particle.x = positions[i] + offset
            particle.m = mass

            if (i < 4):
                particle.f = [100, 0, 0] 

            cubeSystem.particles.append(particle)

        cubeSystem.connection = [[1,2,3,4,5,7],
                    [0,2,3,4,5,6],
                    [0,1,3,5,6,7],
                    [0,1,2,4,6,7],
                    [0,1,3,5,6,7],
                    [0,1,2,4,6,7],
                    [1,2,3,4,5,7],
                    [0,2,3,4,5,6]]

        cubeSystem.initNeighborParticles()
        cubeSystem.origin_positions = cubeSystem.getPositions()

        return cubeSystem

    def makeParticleCloak(self, offset1, offset2):
        neck_point = np.array(offset1) + np.array(offset2)
        neck_point /= 2.

        positions = [[-18., 0., 0.], [-9., 0., 0.], [0., 0., 0.], [9., 0., 0.], [18., 0., 0.],
                    [-26., -10., 0.], [-13., -10., 0.], [0., -10., 0.], [13., -10., 0.], [26., -10., 0.],
                    [-34., -20., 0.], [-17., -20., 0.], [0., -20., 0.], [17., -20., 0.], [34., -20., 0.],
                    [-42., -40., 0.], [-21., -40., 0.], [0., -40., 0.], [21., -40., 0.], [42., -40., 0.],
                    [-50., -60., 0.], [-25., -60., 0.], [0., -60., 0.], [25., -60., 0.], [50., -60., 0.],]
        mass = 10

        cloakSystem = ParticleSystem()

        for i in range(25):
            particle = Particle()
            particle.x = positions[i] + neck_point
            particle.m = mass

            cloakSystem.particles.append(particle)

        cloakSystem.connection = [[1, 5], [0, 2, 6], [1, 3, 7], [2, 4, 8], [3, 9],
                                [0, 6, 10], [1, 5, 7, 11], [2, 6, 8, 12], [3, 7, 9, 13], [4, 8, 14],
                                [5, 11, 15], [6, 10, 12, 16], [7, 11, 13, 17], [8, 12, 14, 18], [9, 13, 19],
                                [10, 16, 20], [11, 15, 17, 21], [12, 16, 18, 22], [13, 17, 19, 23], [14, 18, 24],
                                [15, 21], [16, 20, 22], [17, 21, 23], [18, 22, 24], [19, 23]]

        cloakSystem.initNeighborParticles()
        cloakSystem.origin_positions = cloakSystem.getPositions()

        cloakSystem.stick_point1 = offset1
        cloakSystem.stick_point2 = offset2
        cloakSystem.is_sticked = True

        return cloakSystem

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GLWidget()
    window.show()
    app.exec_()
