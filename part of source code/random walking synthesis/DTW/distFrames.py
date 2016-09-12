import sys
include_path = r'../tools/motion segments/'
sys.path.append(include_path)
from MotionSegment import *
from transformations import *

def distance(worldpos1, worldpos2):
    dist = 0
    for item in worldpos1:
        
        vec1 = np.array(worldpos1[item])
        vec2 = np.array(worldpos2[item])
       
        dist += np.linalg.norm(vec1-vec2)
    return dist

def distFrames(mo1, mo2, weighted = 0):
    # this function measures the distance between two motion sequences
    # input:
    # mo1, mo2 are dictionaries which contain the world position of each frame
    # output:
    # dist: distance between mo1 and mo2
    # angle: rotation anlge about vertical axis from mo1 to mo2 to minimize the distance, in radians
    
    if len(mo1)!= len(mo2):
        raise ValueError('the length of two motion sequences should be equal! ')
    weights = {}
    for joint in mo1[0]:
            if joint == 'root':
                weights[joint] = 11
            elif joint == 'spine_1':
                weights[joint] = 10
            elif joint == 'spine_2':
                weights[joint] = 9
            elif joint == 'spine_3':
                weights[joint] = 8
            elif joint == 'spine_4':
                weights[joint] = 7
            elif joint == 'neck_1':
                weights[joint] = 6
            elif joint == 'neck_2':
                weights[joint] = 5
            elif joint == 'head_ee':
                weights[joint] = 4
            elif joint == 'head_eeEnd':
                weights[joint] = 3
            elif joint == 'left_clavicle':
                weights[joint] = 6
            elif joint == 'left_shoulder':
                weights[joint] = 5
            elif joint == 'left_elbow':
                weights[joint] = 4
            elif joint == 'left_lowarm_twist':
                weights[joint] = 3
            elif joint == 'left_hand':
                weights[joint] = 2
            elif joint == 'left_ee':
                weights[joint] = 1
            elif joint == 'left_eeEnd':
                weights[joint] = 1
            elif joint == 'right_clavicle':
                weights[joint] = 6
            elif joint == 'right_shoulder':
                weights[joint] = 5
            elif joint == 'right_elbow':
                weights[joint] = 4
            elif joint == 'right_lowarm_twist':
                weights[joint] = 3
            elif joint == 'right_hand':
                weights[joint] = 2
            elif joint == 'right_ee':
                weights[joint] = 1
            elif joint == 'right_eeEnd':
                weights[joint] = 1
            elif joint == 'left_hip':
                weights[joint] = 10
            elif joint == 'left_knee':
                weights[joint] = 9
            elif joint == 'left_ankle':
                weights[joint] = 8
            elif joint == 'left_toes':
                weights[joint] = 7
            elif joint == 'left_foot_ee':
                weights[joint] = 6
            elif joint == 'left_foot_eeEnd':
                weights[joint] = 5
            elif joint == 'right_hip':
                weights[joint] = 10
            elif joint == 'right_knee':
                weights[joint] = 9
            elif joint == 'right_ankle':
                weights[joint] = 8
            elif joint == 'right_toes':
                weights[joint] = 7
            elif joint == 'right_foot_ee':
                weights[joint] = 6
            elif joint == 'right_foot_eeEnd':
                weights[joint] = 5
            else:
                print joint
                raise ValueError('Unknown joint')
    skeletonWeights = {}
    if weighted == 0:
        for joint in mo1[0]:
            skeletonWeights[joint] = 1
    else:
        skeletonWeights = weights
        
            
    middleIndex = int(np.floor(len(mo1)/2))
    num = len(mo1)
    # the root position of middle frame of each motion sequence is the origin
    origin1 = np.array(mo1[middleIndex]['root'])
    origin2 = np.array(mo2[middleIndex]['root'])
    # shift all point by origin1 and origin2
    for index in mo1:
        for item in mo1[index]:
            mo1[index][item] = mo1[index][item] - origin1
            mo2[index][item] = mo2[index][item] - origin2
    # estimate the rotation angle, rotate motion1 to motion2
    numerator = 0
    denominator = 0
    for index in mo1:
        for item in mo1[index]:
            numerator += skeletonWeights[item]*(mo1[index][item][0]*mo2[index][item][1] - mo2[index][item][0]*mo1[index][item][1])
            denominator += skeletonWeights[item]*(mo1[index][item][0]*mo2[index][item][0] + mo1[index][item][1]*mo2[index][item][1])
    # the range of theta is [-90 90], however, the rotation angle could be in the range [-180 180]
    # so 180 - theta could be the correct rotation angle
    theta = np.arctan(numerator/denominator)
    theta1 = theta
    if theta >= 0:
        theta2 = theta - np.pi
    else:
        theta2 = theta + np.pi
    # apply rotation with theta1
    # construct rotation matrix
    rotationAxis = [0,0,1]
    rotmat1 = rotation_matrix(theta1, rotationAxis)
    rotatedMo1 = {}
    dist1 = 0
    for index in mo1:
        rotatedMo1[index] = {}
        for item in mo1[index]:
            point = np.ones(4)
            point[:-1] = mo1[index][item]
            newpoint = np.dot(rotmat1, point)
            # measure euclidean distance between corresponding points 
     
            dist1 += weights[item]*np.linalg.norm(newpoint[:-1] - np.array(mo2[index][item]))
            rotatedMo1[index][item] = newpoint.tolist()
    # apply rotation with theta2
    # construct rotation matrix
    rotmat2 = rotation_matrix(theta2, rotationAxis)
    rotatedMo2 = {}
    dist2 = 0
    for index in mo1:
        rotatedMo2[index] = {}
        for item in mo1[index]:
            point = np.ones(4)
            point[:-1] = mo1[index][item]
            newpoint = np.dot(rotmat2, point)
            # measure euclidean distance between corresponding points
            dist2 += weights[item]*np.linalg.norm(newpoint[:-1] - np.array(mo2[index][item]))
            rotatedMo2[index][item] = newpoint.tolist()
    if dist1 > dist2:
        dist = dist2
        theta = theta2
    else:
        dist = dist1
        theta = theta1
    ##
    for index in mo1:
        for item in mo1[index]:
            mo1[index][item] = mo1[index][item] + origin1
            mo2[index][item] = mo2[index][item] + origin2
    return dist/num, theta
        
