# Coordinate-Invariant Distance Function
import numpy as np
joint_set = [
    'root',
    'spine_1',
    'spine_2',
    'spine_3',
    'spine_4',
    'neck_1',
    'neck_2',
    'head_ee',
    'head_eeEnd',
    'left_clavicle',
    'left_shoulder',
    'left_elbow',
    'left_hand',
    'left_ee',
    'left_eeEnd',
    'right_clavicle',
    'right_shoulder',
    'right_elbow',
    'right_hand',
    'right_ee',
    'right_eeEnd',
    'left_hip',
    'left_knee',
    'left_ankle',
    'left_toes',
    'left_foot_ee',
    'left_foot_eeEnd',
    'right_hip',
    'right_knee',
    'right_ankle',
    'right_toes',
    'right_foot_ee',
    'right_foot_eeEnd']

def getDataFromJoint(frame_set):
    # input: frame_set : a dictionary of joint world positions of each frame
    framesData = [] # a list of frame data
    
    for i in frame_set:
        # get the data of each frame
        oneFrame = []
        for joint in joint_set:
            if joint in frame_set[i]:
                
                oneFrame.append(frame_set[i][joint])
        framesData.append(oneFrame)
    return framesData

def FrameDistance(frame_set, index1,index2):
    # FrameDistance implements a coordinate-invariant distance measurement bewteen two frames,
    # frame_set: the frame sequence, frames_set could be a dictionary or a list
    # index1: the index of first frame
    # index2: the index of second frame
    if type(frame_set) is dict:
        framesData = getDataFromJoint(frame_set)
    elif type(frame_set) is list:
        framesData = frame_set
    else:
        print 'incorrenct input data'
    ## extend the framesData
    windowSize = 5
    extendSize = (windowSize-1)/2
    extendedFramesData = []
    for i in range(extendSize):
        extendedFramesData.append(framesData[0])
    extendedFramesData = extendedFramesData + framesData
    for i in range(extendSize):
        extendedFramesData.append(framesData[-1])
    # shift the index to match the new sequence
    index1 = index1 + extendSize
    index2 = index2 + extendSize
    data1 = [] # data of neighbors of frame 1, include itself
    data2 = [] # data of neighbors of frame 2, include itself
    for i in range(index1 - extendSize, index1 + extendSize + 1):
        data1 += extendedFramesData[i]
    for j in range(index2 - extendSize, index2 + extendSize + 1):
        data2 += extendedFramesData[j]
    numData = len(data1)
    ########################################################
    # creating the weight vector
    w = np.ones(numData)
    #w[:21] =0.5
    #print w
    ########################################################
    # compute parameters of transform matrix
    data1 = np.array(data1)
    data2 = np.array(data2)
##    Xmean1,Ymean1, Zmean1 = data1.mean(0)
##    Xmean2,Ymean2, Zmean2 = data2.mean(0)
    Xsum1, Ysum1, Zsum1 = np.sum(data1, 0)
    Xsum2, Ysum2, Zsum2 = np.sum(data2, 0)
    tmp1 = 0
    tmp2 = 0
    for i in range(numData):
        tmp1 += (data1[i][0]*data2[i][1] - data2[i][0]*data1[i][1])* w[i]
        tmp2 += (data1[i][0]*data2[i][0] + data1[i][1]*data2[i][1])* w[i]
    theta = np.arctan((tmp1 - (Xsum1*Ysum2 - Xsum2*Ysum1)/np.sum(w))/(tmp2 - (Xsum1*Xsum2 + Ysum1*Ysum2)/np.sum(w)))
    x0 = (Xsum1 - Xsum2*np.cos(theta) - Ysum2*np.sin(theta))/np.sum(w)
    y0 = (Ysum1 + Xsum2*np.sin(theta) - Ysum2*np.cos(theta))/np.sum(w)

    parameters = [theta, x0, y0]
    # rotation about z (vertical) axis
    rotmat = np.identity(4)
    rotmat[0,0] = np.cos(theta)
    rotmat[0,1] = np.sin(theta)
    rotmat[1,0] = -np.sin(theta)
    rotmat[1,1] = np.cos(theta)
    # transition matrix
    transmat = np.identity(4)
    transmat[0,3] = x0
    transmat[1,3] = y0
    # computer the distance
    distance = 0
    for i in range(numData):
        vec1 = np.ones(4)
        vec1[:-1] = data1[i]
        vec2 = np.ones(4)
        vec2[:-1] = data2[i]
        tmp1 = rotmat*np.mat(vec2).T
        tmp2 = transmat*tmp1
        distance += np.linalg.norm(np.mat(vec1).T - tmp2)

    return distance, parameters
