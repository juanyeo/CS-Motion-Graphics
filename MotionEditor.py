import numpy as np
from Skeleton import Skeleton
from Posture import Posture
from Motion import Motion
import copy
import math
from scipy.spatial.transform import Rotation

def addPostures(posture_1, posture_2):
	combined_posture = copy.deepcopy(posture_1)
	combined_posture.data = []
	combined_posture.rotation_matrix_dict = {}

	for joint in posture_1.rotation_matrix_dict.keys():
		R1 = posture_1.rotation_matrix_dict[joint]
		R2 = posture_2.rotation_matrix_dict[joint]
		combined_posture.rotation_matrix_dict[joint] = R1 @ R2

	combined_posture.root_position = posture_1.root_position
	return combined_posture

def subtractPostures(posture_1, posture_2):
	posture_difference = copy.deepcopy(posture_1)
	posture_difference.data = []
	posture_difference.rotation_matrix_dict = {}

	for joint in posture_1.rotation_matrix_dict.keys():
		R1 = posture_1.rotation_matrix_dict[joint]
		R2 = posture_2.rotation_matrix_dict[joint]

		posture_difference.rotation_matrix_dict[joint] = R1.T @ R2

	posture_difference.root_position = posture_1.root_position
	return posture_difference

def subtractPosturesIgnoreSame(posture_1, posture_2):
	posture_difference = copy.deepcopy(posture_1)
	posture_difference.data = []
	posture_difference.rotation_matrix_dict = {}

	for joint in posture_1.rotation_matrix_dict.keys():
		R1 = posture_1.rotation_matrix_dict[joint]
		R2 = posture_2.rotation_matrix_dict[joint]

		R2_is_identity = sum(R2 @ np.array([1, 2, 3, 4]))
		if R2_is_identity == 10:
			posture_difference.rotation_matrix_dict[joint] = R2
		else:
			posture_difference.rotation_matrix_dict[joint] = R1.T @ R2

	posture_difference.root_position = posture_1.root_position
	return posture_difference

def scalePosture(posture, scale):
	scaled_posture = copy.deepcopy(posture)
	scaled_posture.data = []
	scaled_posture.rotation_matrix_dict = {}

	for joint in posture.rotation_matrix_dict.keys():
		R = posture.rotation_matrix_dict[joint]
		rotation_instance = Rotation.from_matrix(R[:3, :3])
		log_rotation = Rotation.from_rotvec(scale * rotation_instance.as_rotvec())
		scaled_R = np.identity(4)
		scaled_R[:3, :3] = log_rotation.as_matrix()
		scaled_posture.rotation_matrix_dict[joint] = scaled_R

	scaled_posture.root_position = posture.root_position
	return scaled_posture

def makeMotionWarping(motion, posture, warp_at, warp_from, warp_to, type):
    warped_motion = Motion()
    warped_motion = copy.deepcopy(motion)

    if type == "all":
        difference = subtractPostures(motion.posture_list[warp_at], posture)
    else:
        difference = subtractPosturesIgnoreSame(motion.posture_list[warp_at], posture)
    
    scale = list(np.arange(0, 1, 1 / (warp_at - warp_from))) + list(np.arange(1, 0, -1 / (warp_to - warp_at)))
	
    for i in range(len(scale)):
        warped_posture = addPostures(motion.posture_list[warp_from + i], scalePosture(difference, scale[i]))
        warped_motion.posture_list[warp_from + i] = warped_posture

    return warped_motion
