import numpy as np
import copy
from Skeleton import Skeleton
import MatrixUtil as util

class Posture:
    def __init__(self, skeleton, motion_data):
        self.skeleton = skeleton
        self.root_position = np.identity(4)
        self.data = motion_data # angles + positions(ROOT)
        self.affine_matrix_dict = {}
        self.translation_matrix_dict = {}
        self.rotation_matrix_dict = {}
        # calculate initial data
        self.initRootPosition()
        #self.initAffineMatrixDictionary()
        self.initTransAndRotationDictionary()

    def getTranslationMatrix(self, joint):
        return self.translation_matrix_dict[joint.name]

    def getRotationMatrix(self, joint):
        return self.rotation_matrix_dict[joint.name]

    def getAffineMatrix(self, joint):
        return self.affine_matrix_dict[joint.name]

    def getJointPosition(self, name):
        joint = self.skeleton.getJoint(name)

        transformation = np.identity(4)
        while joint is not None:
            #transformation = posture.getAffineMatrix(joint) @ transformation
            transformation = self.getRotationMatrix(joint) @ transformation
            transformation = self.getTranslationMatrix(joint) @ transformation
            joint = joint.parent

        transformation = self.root_position @ transformation

        joint_position = [transformation[0][3], transformation[1][3], transformation[2][3]]
        return joint_position

    def getGlobalRotationMatrix(self, name):
        joint = self.skeleton.getJoint(name)

        if joint.parent == None:
            return np.identity(4)
        return self.getRotationMatrix(joint) @ self.getGlobalRotationMatrix(joint.parent.name)

    def initRootPosition(self):
        position_data = self.data[:3]
        channels = self.skeleton.root.channels[:3]

        self.root_position[:3,3] = [
            position_data[channels.index('Xposition')],
            position_data[channels.index('Yposition')],
            position_data[channels.index('Zposition')]]

    def initTransAndRotationDictionary(self):
        index = 3
        for joint in self.skeleton.joint_list:
            if joint.is_end:
                continue

            channels = joint.channels
            if len(channels) > 3:
                channels = joint.channels[3:] # ROOT
            
            angle_data = self.data[index:index+3]
            
            # 1. Translation <- joint offset
            T = np.identity(4)
            T[:3,3] = joint.offset[:3]
            self.translation_matrix_dict[joint.name] = T

            # 2. Rotation <- angles + channels
            R = np.identity(4)
            R[:3,:3] = util.buildEulerMatrix(angle_data[channels.index('Xrotation')],
                                      angle_data[channels.index('Yrotation')],
                                      angle_data[channels.index('Zrotation')],
                                      channels)

            self.rotation_matrix_dict[joint.name] = R
            index += 3

    def initAffineMatrixDictionary(self):
        index = 3
        for joint in self.skeleton.joint_list:
            if joint.is_end:
                continue

            channels = joint.channels
            if len(channels) > 3:
                channels = joint.channels[3:] # ROOT
            
            angle_data = self.data[index:index+3]
            
            # 1. Translation <- joint offset
            A = np.identity(4)
            A[:3,3] = joint.offset[:3]

            # 2. Rotation <- angles + channels
            R = util.buildEulerMatrix(angle_data[channels.index('Xrotation')],
                                      angle_data[channels.index('Yrotation')],
                                      angle_data[channels.index('Zrotation')],
                                      channels)
            A[:3,:3] = R

            self.affine_matrix_dict[joint.name] = A
            index += 3
    
    