import sys
include_lib = [r'../tools/motion segments/', r'../GMM/',r'../DTW/']
#sys.path.append(include_lib)
sys.path += include_lib
from MotionSegment import *
from MorphableModel import *
from MorphableModel1 import *
from DTW import *
import glob
import re
from sklearn.gaussian_process import GaussianProcess

def getFeatureVector(motionSegment):
    featureVector = []
    for frame in motionSegment.data:
        #featureVector = featureVector + frame
        #selectedFeature = frame[6:]
        selectedFeature = frame[:]
        featureVector = featureVector + selectedFeature
    return featureVector

def diff(vec1, vec2):
    vec1 = np.array(vec1)
    vec2 = np.array(vec2)
    dist = np.linalg.norm(vec1 - vec2)
    orientation = vec2 - vec1
    return dist, orientation
    #return dist

def featureVectorToMotion(featureVector,frameNum):
    motionData = []
    index = 0
    num = len(featureVector)
    frameLen = num/frameNum
    while(index != num):
        if index + frameLen != num:
            motionData.append(featureVector[index:index+frameLen])
        else:
            motionData.append(featureVector[index:])
        index = index + frameLen
    return motionData

def connectMotions(motionList):
    
    num = len(motionList)
    worldpos = motionList[0].worldpos
    for index in range(num-1):
        currentMo = motionList[index]
        nextMo = motionList[index+1]
        diff = np.array(nextMo.data[0][0:3]) - np.array(currentMo.data[-1][0:3])
        for item in nextMo.data:
            temp = np.array(item[0:3]) - diff
            item[0:3] = temp.tolist()
        motion1 = {}
        motion1[0] = currentMo.worldpos[currentMo.numFrames-1]
        motion2 = {}
        motion2[0] = nextMo.worldpos[0]
        dist,theta = distFrames(motion1, motion2)
        print dist
        # rotate the nextMo by angle theta
        angles = np.ones(nextMo.numFrames)
        angles = theta*angles
        nextMo.getMotion(angles)
        # put the pose data into worldpos
        offset = len(worldpos)
        for i in nextMo.worldpos:
            worldpos[offset + i] = nextMo.worldpos[i]
    points = plotPath2D(worldpos)
    dist, orientations = distance(points)
    #print dist
    # generate new motion segment
    # get the inital data
##    data = []
##    for index in range(num):
##        data += motionList[index].data
##    
##    num = len(data)
##    #print num
##    for i in range(num):
##        #data[i][3:6] = data[0][3:6]
##        data[i][1] = data[0][1]
##        if i == 0:
##            data[i][0] = -data[i][6]
##            data[i][2] = -data[i][8]
##        else:
##            data[i][0] = data[0][0]
##            data[i][2] = data[i-1][2] - dist[i-1]
##    newMotion = MotionSegment(data, fileList1[0],type = 'data')
##    print 'called'
##    angles = np.ones(num)
##    theta = -35.0/180.0*np.pi
##    angles = angles*theta
##    newMotion.getMotion(angles)
##    print 'called'
##    print dist
##    print -orientations
##    newMotion.getMotion(-orientations)
    #newMotion.display()
    v = visualizer(motionList[0].hips, worldpos, motionList[0].skeleton)
    v.animate_all()
    #return newMotion

def plotPath2D(worldpos):
    num = len(worldpos)
##    x = np.zeros(num)
##    y = np.zeros(num)
##    z = np.zeros(num)
##    for i in range(num):
##        x[i] = worldpos[i]['root'][0]
##        y[i] = worldpos[i]['root'][1]
##        z[i] = worldpos[i]['root'][2]
##    print z
    points = []
    for i in range(num):
        newpoint = worldpos[i]['root'][0:2]
        points.append(newpoint)
    
    fig = plt.figure()
    plt.plot(*np.transpose(points))
    plt.show()
    return points

def distance(points):
    num = len(points)
    dist = np.zeros(num-1)
    orientations = np.zeros(num)
    for i in range(num-1):
        dist[i], orientation = diff(points[i],points[i+1])
        # compute the orientation angle
        direction = np.array([0,-1])
        # compute the angle(in radians) between two vectors
        #print orientation
        orientations[i] = computeAngle(orientation, direction)
    orientations[-1] = orientations[-2]
    #print orientations
    return dist, orientations

def computeAngle(vec1, vec2):
    if len(vec1) != len(vec2):
        raise ValueError('The length of input vectors should be the same')
    else:
        if np.linalg.norm(vec1) == 0 or np.linalg.norm(vec2):
            angle = 0
        else:
            vec1_u = vec1/np.linalg.norm(vec1)
            vec2_u = vec2/np.linalg.norm(vec2)
            angle = np.arccos(np.dot(vec1_u, vec2_u))
        return angle

def walk(stepNum, num):
    i = 0
    motionList = []
    #num = len(fileList1)
    while i!= stepNum:
        index = np.random.randint(0, num)
        if i%2 == 0:
            if len(motionList) == 0:
                motionList.append(mo_seg1[index])
            else:
                dist = 5000
                while dist>2000:
                    index = np.random.randint(0, num)
                    newMotion = mo_seg1[index]
                    motion1 = {}
                    motion1[0] = motionList[-1].worldpos[motionList[-1].numFrames-1]
                    motion2 = {}
                    motion2[0] = newMotion.worldpos[0]
                    dist, theta = distFrames(motion1, motion2)
                    print dist
                motionList.append(newMotion)
        else:
            if len(motionList) == 0:
                motionList.append(mo_seg2[index])
            else:
                dist = 5000
                while dist>2000:
                    index = np.random.randint(0, num)
                    newMotion = mo_seg2[index]
                    motion1 = {}
                    motion1[0] = motionList[-1].worldpos[motionList[-1].numFrames-1]
                    motion2 = {}
                    motion2[0] = newMotion.worldpos[0]
                    dist, theta = distFrames(motion1, motion2)
                    print dist
                motionList.append(newMotion)
        i = i + 1
    #plotPath2D(motionList.worldpos)
    newMotion = connectMotions(motionList)
    return newMotion
    
    
#filepath1 = r'registered_correlated_pose2/*.bvh'
#filepath2 = r'registered_correlated_pose3/*.bvh'
filepath1 = r'registered_pose2/*.bvh'
filepath2 = r'registered_pose3/*.bvh'
filepath3 = r'registered_correlated_pose3Topose2/*.bvh'
fileList1 = glob.glob(filepath1)
fileList2 = glob.glob(filepath2)
fileList3 = glob.glob(filepath3)
mo_seg1 = []
mo_seg2 = []
mo_seg3 = []
for item in fileList1:
    mo_seg1.append(MotionSegment(item, type = 'file'))
for item in fileList2:
    mo_seg2.append(MotionSegment(item, type = 'file'))
for item in fileList3:
    mo_seg3.append(MotionSegment(item, type = 'file'))

motionList = []
try:
    input_num = int(raw_input('Please input the steps number : '))
except ValueError:
    print "Not a number"
num = len(fileList1)
newMotion = walk(input_num, num)
##i = 0
##while i != input_num:
##    index = np.random.randint(0, num)
##    if i%2 == 0: # if i is a even number
##        motionList.append(mo_seg1[index])
##    else:
##        motionList.append(mo_seg2[index])
##    i = i + 1
#motionList = [mo_seg1[0], mo_seg2[0], mo_seg1[0]]
##connectMotions(motionList)
