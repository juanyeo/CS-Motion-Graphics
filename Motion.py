import numpy as np
from Skeleton import Skeleton
from Posture import Posture

class Motion:
    def __init__(self):
        self.skeleton = None
        self.number_of_frames = 0
        self.fps = 0
        self.posture_list = []
    
    def addPosture(self, frame_number, motion_data):
        posture = Posture(self.skeleton, motion_data)
        self.posture_list.insert(frame_number, posture)
        self.number_of_frames = self.number_of_frames + 1

    def getJointPosition(self, frame_number, name):
        posture = self.posture_list[frame_number]
        joint = self.skeleton.getJoint(name)

        transformation = np.identity(4)
        while joint is not None:
            #transformation = posture.getAffineMatrix(joint) @ transformation
            transformation = posture.getRotationMatrix(joint) @ transformation
            transformation = posture.getTranslationMatrix(joint) @ transformation
            joint = joint.parent

        joint_position = posture.root_position @ transformation
        return joint_position
        
    
