from OpenGL.GL import *
from OpenGL.GLU import *
from GLWidget import GLWidget
from PyQt5 import QtCore, QtGui, QtWidgets
import sys

class Viewer(object):
    def __init__(self, window):
        window.setObjectName("Viewer")
        window.resize(1130, 630)
        self.base_widget = QtWidgets.QWidget(window)
        self.base_widget.setObjectName("base_widget")
        
        self.openGL_widget = GLWidget(self.base_widget)
        self.openGL_widget.setGeometry(QtCore.QRect(0, 0, 1000, 630))
        self.openGL_widget.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.openGL_widget.setObjectName("openGL_widget")

        #self.left_arm_ik_button = QtWidgets.QPushButton(self.base_widget)
        #self.left_arm_ik_button.setGeometry(QtCore.QRect(20, 30, 40, 40))
        #self.left_arm_ik_button.setStyleSheet("border : 3px solid gray;")
        #self.left_arm_ik_button.setObjectName("left_arm_ik_button")
        
        self.group_box = QtWidgets.QGroupBox(self.base_widget)
        self.group_box.setGeometry(QtCore.QRect(20, 510, 960, 150))
        self.group_box.setObjectName("group_box")

        self.play_button = QtWidgets.QPushButton(self.group_box)
        self.play_button.setGeometry(QtCore.QRect(20, 30, 80, 80))
        self.play_button.setStyleSheet("border-radius : 40; border : 3px solid green; color : green")
        self.play_button.setObjectName("play_button")

        self.pause_button = QtWidgets.QPushButton(self.group_box)
        self.pause_button.setGeometry(QtCore.QRect(110, 30, 80, 80))
        self.pause_button.setStyleSheet("border-radius : 40; border : 3px solid red; color : red")
        self.pause_button.setObjectName("pause_button")

        self.frame_slider = QtWidgets.QSlider(self.group_box)
        self.frame_slider.setGeometry(QtCore.QRect(220, 40, 700, 20))
        self.frame_slider.setStyleSheet("border-radius : 10; border : 1px solid white;")
        self.frame_slider.setOrientation(QtCore.Qt.Horizontal)
        self.frame_slider.setObjectName("frame_slider")

        self.frame_spin_box = QtWidgets.QSpinBox(self.group_box)
        self.frame_spin_box.setGeometry(QtCore.QRect(850, 70, 60, 26))
        self.frame_spin_box.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.frame_spin_box.setObjectName("frame_spin_box")

        self.right_box = QtWidgets.QGroupBox(self.base_widget)
        self.right_box.setGeometry(QtCore.QRect(980, 0, 150, 650))
        self.right_box.setStyleSheet("background : white")
        self.right_box.setObjectName("right_box")

        self.ik_title = QtWidgets.QLabel(self.right_box)
        self.ik_title.setGeometry(QtCore.QRect(0, 0, 150, 30))
        self.ik_title.setText("Limb IK")
        self.ik_title.setAlignment(QtCore.Qt.AlignCenter)
        self.ik_title.setStyleSheet("background : white; font-size: 18pt; color:orange; font-weight: 600;")
        self.ik_title.setObjectName("ik_title")

        self.top_image = QtWidgets.QLabel(self.right_box)
        self.top_image.setGeometry(QtCore.QRect(0, 35, 150, 226))
        self.top_image.setPixmap(QtGui.QPixmap("./assets/human.png"))
        self.top_image.setObjectName("top_image")

        self.right_arm = QtWidgets.QPushButton(self.right_box)
        self.right_arm.setGeometry(QtCore.QRect(16, 170, 36, 36))
        self.right_arm.setStyleSheet("border : 3px solid gray; background : transparent")
        self.right_arm.setObjectName("right_arm")

        self.left_arm = QtWidgets.QPushButton(self.right_box)
        self.left_arm.setGeometry(QtCore.QRect(100, 170, 36, 36))
        self.left_arm.setStyleSheet("border : 3px solid gray; background : transparent")
        self.left_arm.setObjectName("left_arm")

        self.right_leg = QtWidgets.QPushButton(self.right_box)
        self.right_leg.setGeometry(QtCore.QRect(36, 220, 36, 36))
        self.right_leg.setStyleSheet("border : 3px solid gray; background : transparent")
        self.right_leg.setObjectName("right_leg")

        self.left_leg = QtWidgets.QPushButton(self.right_box)
        self.left_leg.setGeometry(QtCore.QRect(80, 220, 36, 36))
        self.left_leg.setStyleSheet("border : 3px solid gray; background : transparent")
        self.left_leg.setObjectName("left_leg")

        self.x_label = QtWidgets.QLabel(self.right_box)
        self.x_label.setGeometry(QtCore.QRect(10, 270, 30, 36))
        self.x_label.setText("X : ")
        self.x_label.setObjectName("x_label")
        self.x_spin_box = QtWidgets.QDoubleSpinBox(self.right_box)
        self.x_spin_box.setGeometry(QtCore.QRect(50, 270, 90, 36))
        self.x_spin_box.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.x_spin_box.setRange(-1000, 1000)
        self.x_spin_box.setObjectName("x_spin_box")

        self.y_label = QtWidgets.QLabel(self.right_box)
        self.y_label.setGeometry(QtCore.QRect(10, 316, 30, 36))
        self.y_label.setText("Y : ")
        self.y_label.setObjectName("y_label")
        self.y_spin_box = QtWidgets.QDoubleSpinBox(self.right_box)
        self.y_spin_box.setGeometry(QtCore.QRect(50, 316, 90, 36))
        self.y_spin_box.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.y_spin_box.setRange(-1000, 1000)
        self.y_spin_box.setObjectName("y_spin_box")

        self.z_label = QtWidgets.QLabel(self.right_box)
        self.z_label.setGeometry(QtCore.QRect(10, 362, 30, 36))
        self.z_label.setText("Z : ")
        self.z_label.setObjectName("z_label")
        self.z_spin_box = QtWidgets.QDoubleSpinBox(self.right_box)
        self.z_spin_box.setGeometry(QtCore.QRect(50, 362, 90, 36))
        self.z_spin_box.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.z_spin_box.setRange(-1000, 1000)
        self.z_spin_box.setObjectName("z_spin_box")

        self.warping_title = QtWidgets.QLabel(self.right_box)
        self.warping_title.setGeometry(QtCore.QRect(0, 410, 150, 30))
        self.warping_title.setText("Warping")
        self.warping_title.setAlignment(QtCore.Qt.AlignCenter)
        self.warping_title.setStyleSheet("background : white; font-size: 18pt; color:green; font-weight: 600;")
        self.warping_title.setObjectName("warping_title")

        self.f_label = QtWidgets.QLabel(self.right_box)
        self.f_label.setGeometry(QtCore.QRect(10, 450, 50, 36))
        self.f_label.setText("Frame : ")
        self.f_label.setObjectName("f_label")
        self.f_spin_box = QtWidgets.QSpinBox(self.right_box)
        self.f_spin_box.setGeometry(QtCore.QRect(70, 450, 70, 36))
        self.f_spin_box.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.f_spin_box.setRange(0, 10000)
        self.f_spin_box.setObjectName("f_spin_box")

        #self.sharp_label = QtWidgets.QLabel(self.right_box)
        #self.sharp_label.setGeometry(QtCore.QRect(15, 500, 100, 20))
        #self.sharp_label.setText("<- Sharpness ->")
        #self.sharp_label.setObjectName("sharp_label")
        self.sharp_slider = QtWidgets.QSlider(self.right_box)
        self.sharp_slider.setGeometry(QtCore.QRect(15, 510, 120, 30))
        self.sharp_slider.setStyleSheet("border-radius : 10; border : 1px solid white;")
        self.sharp_slider.setOrientation(QtCore.Qt.Horizontal)
        self.sharp_slider.setObjectName("sharp_slider")

        self.warp_button = QtWidgets.QPushButton(self.right_box)
        self.warp_button.setGeometry(QtCore.QRect(75, 560, 60, 60))
        self.warp_button.setText("OFF")
        self.warp_button.setStyleSheet("border-radius : 30; background : gray; color : white; font-weight: 600")
        self.warp_button.setObjectName("warp_button")

        self.warp_type_check = QtWidgets.QCheckBox(self.right_box)
        self.warp_type_check.setGeometry(QtCore.QRect(15, 590, 60, 30))
        self.warp_type_check.setText("ALL")
        self.warp_type_check.toggle()
        self.warp_type_check.setStyleSheet("background : transparent; color : green;")
        self.warp_type_check.setObjectName("warp_type_check")

        window.setCentralWidget(self.base_widget)
        self.timer = QtCore.QTimer()
        self.retranslateUi(window)
        QtCore.QMetaObject.connectSlotsByName(window)

        self.play_button.clicked.connect(self.onClickPlayButton)
        self.pause_button.clicked.connect(self.onClickPauseButton)
        #self.left_arm_ik_button.clicked.connect(self.onClickLeftArmIKButton)
        self.right_arm.clicked.connect(self.onClickRightArm)
        self.left_arm.clicked.connect(self.onClickLeftArm)
        self.right_leg.clicked.connect(self.onClickRightLeg)
        self.left_leg.clicked.connect(self.onClickLeftLeg)
        self.warp_button.clicked.connect(self.onClickWarpButton)
        self.warp_type_check.stateChanged.connect(self.onClickWarpTypeCheck)
        self.setSlider()
        self.setSpinBox()
        self.setTimer()

    def retranslateUi(self, window):
        application_translate = QtCore.QCoreApplication.translate
        window.setWindowTitle(application_translate("Viewer", "[Juan] Motion Viewer"))
        self.play_button.setText(application_translate("Viewer", "PLAY"))
        self.pause_button.setText(application_translate("Viewer", "PAUSE"))

    def onClickPlayButton(self):
        self.frame_slider.setRange(0, self.openGL_widget.number_of_frames - 1)
        self.frame_spin_box.setRange(0, self.openGL_widget.number_of_frames - 1)
        if self.openGL_widget.current_frame > self.openGL_widget.number_of_frames:
            self.openGL_widget.current_frame = 0
            self.openGL_widget.run = True
            self.openGL_widget.repaint()
        self.timer.start()

    def onClickPauseButton(self):
        self.timer.stop()

    def onClickWarpButton(self):
        if self.openGL_widget.warp_run:
            self.warp_button.setText("OFF")
            self.warp_button.setStyleSheet("border-radius : 30; background : gray; color : white; font-weight: 600")
        else:
            self.warp_button.setText("ON")
            self.warp_button.setStyleSheet("border-radius : 30; background : green; color : white; font-weight: 600")
        self.openGL_widget.initializeMotionWarp()

    def onClickWarpTypeCheck(self, state):
        if state == QtCore.Qt.Checked:
            self.openGL_widget.warp_type = "all"
        else:
            self.openGL_widget.warp_type = "part"
    
    def onClickLeftArmIKButton(self):
        if self.openGL_widget.mode_ik == 0:
            self.openGL_widget.mode_ik = 1
            #self.left_arm_ik_button.setStyleSheet("border : 3px solid orange;")
        elif self.openGL_widget.mode_ik == 1:
            self.openGL_widget.mode_ik = 0
            #self.left_arm_ik_button.setStyleSheet("border : 3px solid gray;")

    def setTimer(self):
        self.timer.timeout.connect(self.timeout)
        self.timer.setInterval(1000 / self.openGL_widget.fps)
    
    def timeout(self):
        self.openGL_widget.current_frame += 1
        if self.openGL_widget.current_frame > self.openGL_widget.number_of_frames:
            self.openGL_widget.current_frame = 0
        self.frame_slider.setValue(self.openGL_widget.current_frame)

    def setSlider(self):
        slider = self.frame_slider
        slider.setRange(0, self.openGL_widget.number_of_frames - 1)
        slider.setSingleStep(1)
        slider.valueChanged.connect(self.moveSlider)

        sharp_slider = self.sharp_slider
        sharp_slider.setRange(10, 100)
        sharp_slider.setSingleStep(1)
        sharp_slider.setValue(50)
        sharp_slider.valueChanged.connect(self.moveSharpSlider)

    def moveSlider(self):
        self.openGL_widget.current_frame = self.frame_slider.value()
        self.frame_spin_box.setValue(self.frame_slider.value())
        self.openGL_widget.run = True
        self.openGL_widget.repaint()

    def moveSharpSlider(self):
        self.openGL_widget.warp_interval = self.sharp_slider.value()
        if self.openGL_widget.warp_run:
            self.openGL_widget.updateMotionWarp()

    def setSpinBox(self):
        spin_box = self.frame_spin_box
        spin_box.setRange(0, self.openGL_widget.number_of_frames - 1)
        spin_box.setSingleStep(1)
        spin_box.valueChanged.connect(self.changeSpinBox)

        x_spin_box = self.x_spin_box
        x_spin_box.setSingleStep(1)
        x_spin_box.valueChanged.connect(self.changeXSpinBox)

        y_spin_box = self.y_spin_box
        y_spin_box.setSingleStep(1)
        y_spin_box.valueChanged.connect(self.changeYSpinBox)

        z_spin_box = self.z_spin_box
        z_spin_box.setSingleStep(1)
        z_spin_box.valueChanged.connect(self.changeZSpinBox)

        f_spin_box = self.f_spin_box
        f_spin_box.setSingleStep(1)
        f_spin_box.valueChanged.connect(self.changeFSpinBox)

    def changeSpinBox(self):
        self.openGL_widget.current_frame = self.frame_spin_box.value()
        self.frame_slider.setValue(self.frame_spin_box.value())

    def changeXSpinBox(self):
        self.openGL_widget.calculateIKVectors([self.x_spin_box.value(), self.y_spin_box.value(), self.z_spin_box.value()])

    def changeYSpinBox(self):
        self.openGL_widget.calculateIKVectors([self.x_spin_box.value(), self.y_spin_box.value(), self.z_spin_box.value()])

    def changeZSpinBox(self):
        self.openGL_widget.calculateIKVectors([self.x_spin_box.value(), self.y_spin_box.value(), self.z_spin_box.value()])

    def changeFSpinBox(self):
        self.openGL_widget.warp_frame = self.f_spin_box.value()
        if self.openGL_widget.warp_run:
            self.openGL_widget.updateMotionWarp()

    def initXYZSpinBox(self, joint_name):
        edge_point = self.openGL_widget.ik_posture.getJointPosition(joint_name)
        self.openGL_widget.ik_edge_point = edge_point
        self.x_spin_box.blockSignals(True)
        self.y_spin_box.blockSignals(True)
        self.z_spin_box.blockSignals(True)
        self.x_spin_box.setValue(edge_point[0])
        self.y_spin_box.setValue(edge_point[1])
        self.z_spin_box.setValue(edge_point[2])
        self.x_spin_box.blockSignals(False)
        self.y_spin_box.blockSignals(False)
        self.z_spin_box.blockSignals(False)

    def onClickRightArm(self):
        if self.openGL_widget.mode_ik == 1:
            self.openGL_widget.mode_ik = 0
            self.right_arm.setStyleSheet("border : 3px solid gray; background : transparent")
        else:
            self.openGL_widget.mode_ik = 1
            self.right_arm.setStyleSheet("border : 3px solid orange; background : transparent")
            self.left_arm.setStyleSheet("border : 3px solid gray; background : transparent")
            self.right_leg.setStyleSheet("border : 3px solid gray; background : transparent")
            self.left_leg.setStyleSheet("border : 3px solid gray; background : transparent")
            self.initXYZSpinBox("rHand")
    
    def onClickLeftArm(self):
        if self.openGL_widget.mode_ik == 2:
            self.openGL_widget.mode_ik = 0
            self.left_arm.setStyleSheet("border : 3px solid gray; background : transparent")
        else:
            self.openGL_widget.mode_ik = 2
            self.right_arm.setStyleSheet("border : 3px solid gray; background : transparent")
            self.left_arm.setStyleSheet("border : 3px solid orange; background : transparent")
            self.right_leg.setStyleSheet("border : 3px solid gray; background : transparent")
            self.left_leg.setStyleSheet("border : 3px solid gray; background : transparent")
            self.initXYZSpinBox("lHand")

    def onClickRightLeg(self):
        if self.openGL_widget.mode_ik == 3:
            self.openGL_widget.mode_ik = 0
            self.right_leg.setStyleSheet("border : 3px solid gray; background : transparent")
        else:
            self.openGL_widget.mode_ik = 3
            self.right_arm.setStyleSheet("border : 3px solid gray; background : transparent")
            self.left_arm.setStyleSheet("border : 3px solid gray; background : transparent")
            self.right_leg.setStyleSheet("border : 3px solid orange; background : transparent")
            self.left_leg.setStyleSheet("border : 3px solid gray; background : transparent")
            self.initXYZSpinBox("rFoot")

    def onClickLeftLeg(self):
        if self.openGL_widget.mode_ik == 4:
            self.openGL_widget.mode_ik = 0
            self.left_leg.setStyleSheet("border : 3px solid gray; background : transparent")
        else:
            self.openGL_widget.mode_ik = 4
            self.right_arm.setStyleSheet("border : 3px solid gray; background : transparent")
            self.left_arm.setStyleSheet("border : 3px solid gray; background : transparent")
            self.right_leg.setStyleSheet("border : 3px solid gray; background : transparent")
            self.left_leg.setStyleSheet("border : 3px solid orange; background : transparent")
            self.initXYZSpinBox("lFoot")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = QtWidgets.QMainWindow()
    viewer = Viewer(window)
    window.show()
    sys.exit(app.exec_())
        
        