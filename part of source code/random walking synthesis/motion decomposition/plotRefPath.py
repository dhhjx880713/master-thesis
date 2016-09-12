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

def main():
    ref_file = r'../motion decomposition/registered_pose2/registered_61.bvh'
    mo = MotionSegment(ref_file, type = 'file')
    plotPath2D(mo.worldpos)

def randomSample():
    filepath = r'../motion decomposition/registered_pose2/*.bvh'
    #filepath = r'../motion decomposition/pose2/*.bvh'
    fileList = glob.glob(filepath)
    num = len(fileList)
    index = np.random.randint(0,num)
    print index
    mo = MotionSegment(fileList[index], type = 'file')
    plotPath2D(mo.worldpos)

randomSample()
