import numpy as np
import re
from Joint import Joint
from Skeleton import Skeleton
from Motion import Motion

file_path = './motion_files/85_02.bvh'

class BvhParser:
    def __init__(self, path):
        # parse_skeleton
        self.root = None
        self.joints = []
        # parse_motion
        self.number_of_frames = 0
        self.fps = 0
        self.frame_motions = None
        self.parseBvhFile(path)

    def parseBvhFile(self, path):
        with open(path, 'r') as f:
            text = f.read()
            skeleton_text, motion_text = text.split('MOTION')
            self.parseSkeleton(skeleton_text)
            self.parseMotion(motion_text)

    def parseSkeleton(self, text):
        lines = re.split('\\s*\\n+\\s*', text)
        joint_stack = []

        for line in lines:
            words = re.split('\\s+', line)
            key = words[0]

            if key == "ROOT":
                joint = Joint(words[1])
                self.root = joint
                joint_stack.append(joint)
            elif key == "JOINT" or key == "End":
                joint_name = words[1] if key == "JOINT" else "end_" + joint_stack[-1].name
                joint = Joint(joint_name)
                if key == "End":
                    joint.is_end = True
                parent = joint_stack[-1]
                if parent:
                    joint.parent = parent
                    parent.addChild(joint)
                joint_stack.append(joint)
            elif key == "OFFSET":
                joint_stack[-1].offset[0] = float(words[1])
                joint_stack[-1].offset[1] = float(words[2])
                joint_stack[-1].offset[2] = float(words[3])
            elif key == "CHANNELS":
                for i in range(2, len(words)):
                    joint_stack[-1].channels.append(words[i])
            elif key == "}":
                if joint_stack:
                    self.joints.append(joint_stack.pop())

    def parseMotion(self, text):
        lines = re.split('\\s*\\n+\\s*', text)
        
        frame_number = 0
        for line in lines:
            if line == '':
                continue
            words = re.split('\\s+', line)
            
            if line.startswith("Frame Time:"):
                self.fps = round(1 / float(words[2]))
                continue
            elif line.startswith("Frames:"):
                self.number_of_frames = int(words[1])
                continue

            # initialize frame_motions (2D Array)
            if self.frame_motions is None:
                self.frame_motions = np.empty((self.number_of_frames, len(words)), dtype=np.float32)
            
            self.frame_motions[frame_number][:] = [float(angle) for angle in words]
            frame_number += 1
    
    def getMotion(self, start=0, end=0):
        if end == 0:
            end = self.number_of_frames

        motion = Motion()
        motion.skeleton = Skeleton(self.root)
        motion.fps = self.fps
        for frame_number in range(start, end):
            motion.addPosture(frame_number, self.frame_motions[frame_number])
        
        return motion

parser = BvhParser(file_path)
