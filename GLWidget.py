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
        height = self.motion.posture_list[0].data[1]
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GLWidget()
    window.show()
    app.exec_()
