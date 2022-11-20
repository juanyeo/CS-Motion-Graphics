import numpy as np
import math
from scipy.spatial.transform import Rotation
from pyquaternion import Quaternion

def buildEulerMatrix(x_angle, y_angle, z_angle, channels):
    rotate_order = [0, 0, 0]

    Rx = buildRotationMatrixX(x_angle)
    Ry = buildRotationMatrixY(y_angle)
    Rz = buildRotationMatrixZ(z_angle)

    rotate_order[channels.index('Xrotation')] = Rx
    rotate_order[channels.index('Yrotation')] = Ry
    rotate_order[channels.index('Zrotation')] = Rz

    R = rotate_order[0] @ rotate_order[1] @ rotate_order[2]
    return R

def buildRotationMatrixX(angle):
    angle *= np.pi/180
    rotate_matrix = [[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]]
    return np.array(rotate_matrix)

def buildRotationMatrixY(angle):
    angle *= np.pi/180
    rotate_matrix = [[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]]
    return np.array(rotate_matrix)

def buildRotationMatrixZ(angle):
    angle *= np.pi/180
    rotate_matrix = [[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]]
    return np.array(rotate_matrix)

def calculateInverseKinematics(a, b, c, t):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    t = np.array(t)
    
    lab = np.linalg.norm(a-b)
    lbc = np.linalg.norm(b-c)
    lac = np.linalg.norm(a-c)
    lat = np.linalg.norm(a-t)
    lct = np.linalg.norm(c-t)

    alpha = math.acos(np.clip((lab**2 + lac**2 - lbc**2) / (2*lab*lac), -1, 1))
    new_alpha = math.acos(np.clip((lab**2 + lat**2 - lbc**2) / (2*lab*lat), -1, 1))

    beta = math.acos(np.clip((lab**2 + lbc**2 - lac**2) / (2*lab*lbc), -1, 1))
    new_beta = math.acos(np.clip((lab**2 + lbc**2 - lat**2) / (2*lab*lbc), -1, 1))

    gamma = math.acos(np.clip((lat**2 + lac**2 - lct**2) / (2*lat*lac), -1, 1))

    axis_a = np.cross(c - a, b - a)
    if np.linalg.norm(axis_a) != 0.:
        axis_a = axis_a / np.linalg.norm(axis_a)
    axis_g = np.cross(c - a, t - a)
    if np.linalg.norm(axis_g) != 0.:
        axis_g = axis_g / np.linalg.norm(axis_g)
    
    alpha_prime = alpha - new_alpha
    cos_a = np.cos(alpha_prime)
    sin_a = np.sin(alpha_prime)
    Ra = [[cos_a+(1-cos_a)*(axis_a[0]**2), (1-cos_a)*axis_a[0]*axis_a[1]-axis_a[2]*sin_a, (1-cos_a)*axis_a[0]*axis_a[2]+axis_a[1]*sin_a, 0.],
            [(1-cos_a)*axis_a[0]*axis_a[1]+axis_a[2]*sin_a, cos_a+(1-cos_a)*(axis_a[1]**2), (1-cos_a)*axis_a[1]*axis_a[2]-axis_a[0]*sin_a, 0.],
            [(1-cos_a)*axis_a[0]*axis_a[2]-axis_a[1]*sin_a, (1-cos_a)*axis_a[1]*axis_a[2]+axis_a[0]*sin_a, cos_a+(1-cos_a)*(axis_a[2]**2), 0.],
            [0., 0., 0., 1.]]
    
    beta_prime = beta - new_beta
    cos_b = np.cos(beta_prime)
    sin_b = np.sin(beta_prime)
    Rb = [[cos_b+(1-cos_b)*(axis_a[0]**2), (1-cos_b)*axis_a[0]*axis_a[1]-axis_a[2]*sin_b, (1-cos_b)*axis_a[0]*axis_a[2]+axis_a[1]*sin_b, 0.],
            [(1-cos_b)*axis_a[0]*axis_a[1]+axis_a[2]*sin_b, cos_b+(1-cos_b)*(axis_a[1]**2), (1-cos_b)*axis_a[1]*axis_a[2]-axis_a[0]*sin_b, 0.],
            [(1-cos_b)*axis_a[0]*axis_a[2]-axis_a[1]*sin_b, (1-cos_b)*axis_a[1]*axis_a[2]+axis_a[0]*sin_b, cos_b+(1-cos_b)*(axis_a[2]**2), 0.],
            [0., 0., 0., 1.]]
    
    cos_g = np.cos(gamma)
    sin_g = np.sin(gamma)
    Rg = [[cos_g+(1-cos_g)*(axis_g[0]**2), (1-cos_g)*axis_g[0]*axis_g[1]-axis_g[2]*sin_g, (1-cos_g)*axis_g[0]*axis_g[2]+axis_g[1]*sin_g, 0.],
            [(1-cos_g)*axis_g[0]*axis_g[1]+axis_g[2]*sin_g, cos_g+(1-cos_g)*(axis_g[1]**2), (1-cos_g)*axis_g[1]*axis_g[2]-axis_g[0]*sin_g, 0.],
            [(1-cos_g)*axis_g[0]*axis_g[2]-axis_g[1]*sin_g, (1-cos_g)*axis_g[1]*axis_g[2]+axis_g[0]*sin_g, cos_g+(1-cos_g)*(axis_g[2]**2), 0.],
            [0., 0., 0., 1.]]

    return Ra, Rb, Rg

def calculateInverseKinematics2(a, b, c, t):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    t = np.array(t)
    
    lab = np.linalg.norm(a-b)
    lbc = np.linalg.norm(b-c)
    lat = np.clip(np.linalg.norm(a-t), 0.01, lab + lbc -0.01)

    alpha = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (b - a) / np.linalg.norm(b - a)), -1, 1))
    beta = math.acos(np.clip(np.dot((a - b) / np.linalg.norm(a - b), (c - b) / np.linalg.norm(c - b)), -1, 1))
    gamma = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (t - a) / np.linalg.norm(t - a)), -1, 1))
    
    new_alpha = math.acos(np.clip((lbc * lbc - lab * lab - lat * lat) / (-2 * lab * lat), -1, 1))
    new_beta = math.acos(np.clip((lat * lat - lab * lab - lbc * lbc) / (-2 * lab * lbc), -1, 1))

    axis_a = np.cross(c - a, b - a)
    axis_a = axis_a / np.linalg.norm(axis_a)
    axis_g = np.cross(c - a, t - a)
    axis_g = axis_g / np.linalg.norm(axis_g)
    
    alpha_prime = new_alpha - alpha
    cos_a = np.cos(alpha_prime)
    sin_a = np.sin(alpha_prime)
    Ra = [[cos_a+(1-cos_a)*(axis_a[0]**2), (1-cos_a)*axis_a[0]*axis_a[1]-axis_a[2]*sin_a, (1-cos_a)*axis_a[0]*axis_a[2]+axis_a[1]*sin_a, 0.],
            [(1-cos_a)*axis_a[0]*axis_a[1]+axis_a[2]*sin_a, cos_a+(1-cos_a)*(axis_a[1]**2), (1-cos_a)*axis_a[1]*axis_a[2]-axis_a[0]*sin_a, 0.],
            [(1-cos_a)*axis_a[0]*axis_a[2]-axis_a[1]*sin_a, (1-cos_a)*axis_a[1]*axis_a[2]+axis_a[0]*sin_a, cos_a+(1-cos_a)*(axis_a[2]**2), 0.],
            [0., 0., 0., 1.]]
    
    beta_prime = new_beta - beta
    cos_b = np.cos(beta_prime)
    sin_b = np.sin(beta_prime)
    Rb = [[cos_b+(1-cos_b)*(axis_a[0]**2), (1-cos_b)*axis_a[0]*axis_a[1]-axis_a[2]*sin_b, (1-cos_b)*axis_a[0]*axis_a[2]+axis_a[1]*sin_b, 0.],
            [(1-cos_b)*axis_a[0]*axis_a[1]+axis_a[2]*sin_b, cos_b+(1-cos_b)*(axis_a[1]**2), (1-cos_b)*axis_a[1]*axis_a[2]-axis_a[0]*sin_b, 0.],
            [(1-cos_b)*axis_a[0]*axis_a[2]-axis_a[1]*sin_b, (1-cos_b)*axis_a[1]*axis_a[2]+axis_a[0]*sin_b, cos_b+(1-cos_b)*(axis_a[2]**2), 0.],
            [0., 0., 0., 1.]]
    
    cos_g = np.cos(gamma)
    sin_g = np.sin(gamma)
    Rg = [[cos_g+(1-cos_g)*(axis_g[0]**2), (1-cos_g)*axis_g[0]*axis_g[1]-axis_g[2]*sin_g, (1-cos_g)*axis_g[0]*axis_g[2]+axis_g[1]*sin_g, 0.],
            [(1-cos_g)*axis_g[0]*axis_g[1]+axis_g[2]*sin_g, cos_g+(1-cos_g)*(axis_g[1]**2), (1-cos_g)*axis_g[1]*axis_g[2]-axis_g[0]*sin_g, 0.],
            [(1-cos_g)*axis_g[0]*axis_g[2]-axis_g[1]*sin_g, (1-cos_g)*axis_g[1]*axis_g[2]+axis_g[0]*sin_g, cos_g+(1-cos_g)*(axis_g[2]**2), 0.],
            [0., 0., 0., 1.]]

    return Ra, Rb, Rg

def calculateInverseKinematics3(a, b, c, t, a_gr, b_gr, a_lr, b_lr):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    t = np.array(t)
    print(c, t)
    
    lab = np.linalg.norm(b-a)
    lbc = np.linalg.norm(b-c)
    lat = np.clip(np.linalg.norm(t-a), 0.01, lab + lbc -0.01)

    alpha = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (b - a) / np.linalg.norm(b - a)), -1, 1))
    beta = math.acos(np.clip(np.dot((a - b) / np.linalg.norm(a - b), (c - b) / np.linalg.norm(c - b)), -1, 1))
    gamma = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (t - a) / np.linalg.norm(t - a)), -1, 1))
    
    new_alpha = math.acos(np.clip((lbc * lbc - lab * lab - lat * lat) / (-2 * lab * lat), -1, 1))
    new_beta = math.acos(np.clip((lat * lat - lab * lab - lbc * lbc) / (-2 * lab * lbc), -1, 1))

    axis_a = np.cross(c - a, b - a)
    axis_a = axis_a / np.linalg.norm(axis_a)
    axis_g = np.cross(c - a, t - a)
    axis_g = axis_g / np.linalg.norm(axis_g)

    r0 = Rotation.from_rotvec(a_gr[:3,:3].T @ axis_a * (new_alpha - alpha)).as_matrix()
    r1 = Rotation.from_rotvec(b_gr[:3,:3].T @ axis_a * (new_beta - beta)).as_matrix()
    r2 = Rotation.from_rotvec(a_gr[:3,:3].T @ axis_g * (gamma)).as_matrix()

    new_a_lr = np.identity(4)
    new_a_lr[:3,:3] = a_lr[:3,:3] @ r0 @ r2
    new_b_lr = np.identity(4)
    new_b_lr[:3,:3] = b_lr[:3,:3] @ r1

    return new_a_lr, new_b_lr

def calculateInverseKinematics4(a, b, c, t, a_gr, b_gr, a_lr, b_lr):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    t = np.array(t)
    print(c, t)
    
    lab = np.linalg.norm(a-b)
    lbc = np.linalg.norm(b-c)
    lat = np.clip(np.linalg.norm(a-t), 0.01, lab + lbc -0.01)

    alpha = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (b - a) / np.linalg.norm(b - a)), -1, 1))
    beta = math.acos(np.clip(np.dot((a - b) / np.linalg.norm(a - b), (c - b) / np.linalg.norm(c - b)), -1, 1))
    gamma = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (t - a) / np.linalg.norm(t - a)), -1, 1))
    
    new_alpha = math.acos(np.clip((lbc * lbc - lab * lab - lat * lat) / (-2 * lab * lat), -1, 1))
    new_beta = math.acos(np.clip((lat * lat - lab * lab - lbc * lbc) / (-2 * lab * lbc), -1, 1))

    axis_a = np.cross(c - a, b - a)
    axis_a = axis_a / np.linalg.norm(axis_a)
    axis_g = np.cross(c - a, t - a)
    axis_g = axis_g / np.linalg.norm(axis_g)
    
    r0 = matrix_from_axis_angle(a_gr.T @ np.append(axis_a, 0), new_alpha - alpha)
    r1 = matrix_from_axis_angle(b_gr.T @ np.append(axis_a, 0), new_beta - beta)
    r2 = matrix_from_axis_angle(a_gr.T @ np.append(axis_g, 0), gamma)

    new_a_lr = a_lr @ (r0 @ r2)
    new_b_lr = b_lr @ r1

    return new_a_lr, new_b_lr

def calculateInverseKinematics5(a, b, c, t, posture, joint_a, joint_b):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    t = np.array(t)
    print(c, t)
    
    lab = np.linalg.norm(a-b)
    lbc = np.linalg.norm(b-c)
    lat = np.clip(np.linalg.norm(a-t), 0.01, lab + lbc -0.01)

    alpha = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (b - a) / np.linalg.norm(b - a)), -1, 1))
    beta = math.acos(np.clip(np.dot((a - b) / np.linalg.norm(a - b), (c - b) / np.linalg.norm(c - b)), -1, 1))
    gamma = math.acos(np.clip(np.dot((c - a) / np.linalg.norm(c - a), (t - a) / np.linalg.norm(t - a)), -1, 1))
    
    new_alpha = math.acos(np.clip((lbc * lbc - lab * lab - lat * lat) / (-2 * lab * lat), -1, 1))
    new_beta = math.acos(np.clip((lat * lat - lab * lab - lbc * lbc) / (-2 * lab * lbc), -1, 1))

    a_gr = posture.getGlobalRotationMatrix(joint_a)
    a_lr = posture.rotation_matrix_dict[joint_a]

    axis_a = np.cross(c - a, b - a)
    axis_a = axis_a / np.linalg.norm(axis_a)
    axis_g = np.cross(c - a, t - a)
    axis_g = axis_g / np.linalg.norm(axis_g)
    
    r0 = matrix_from_axis_angle(a_gr.T @ np.append(axis_a, 0), new_alpha - alpha)
    r2 = matrix_from_axis_angle(a_gr.T @ np.append(axis_g, 0), gamma)

    new_a_lr = a_lr @ (r0 @ r2)

    posture.rotation_matrix_dict[joint_a] = new_a_lr

    b_gr = posture.getGlobalRotationMatrix(joint_b)
    b_lr = posture.rotation_matrix_dict[joint_b]

    r1 = matrix_from_axis_angle(b_gr.T @ np.append(axis_a, 0), new_beta - beta)

    new_b_lr = b_lr @ r1

    posture.rotation_matrix_dict[joint_b] = new_b_lr

    return posture

def matrix_from_axis_angle(axis, degree):
    x = axis[0] * np.sin(degree / 2)
    y = axis[1] * np.sin(degree / 2)
    z = axis[2] * np.sin(degree / 2)
    c = np.cos(degree / 2)

    rotation_matrix = np.identity(4)
    rotation_matrix[:3, :3] = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y + 2*c*z, 2*x*z - 2*c*y],
        [2*x*y - 2*c*z, 1 - 2*x*x -2*z*z, 2*y*z + 2*c*x],
        [2*x*z + 2*c*y, 2*y*z - 2*c*x, 1 - 2*x*x - 2*y*y]])

    return rotation_matrix