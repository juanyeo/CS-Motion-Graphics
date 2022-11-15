class Skeleton:
    def __init__(self, root):
        self.root = root
        self.joint_list = self.sortJointList(root)

    def sortJointList(self, root):
        distinct_list = []
        stack = [root]

        while stack:
            joint = stack.pop()
            distinct_list.append(joint)
            for child_joint in reversed(joint.children):
                stack.append(child_joint)
        
        return distinct_list
    
    def getJoint(self, name):
        for joint in self.joint_list:
            if joint.name == name:
                return joint
