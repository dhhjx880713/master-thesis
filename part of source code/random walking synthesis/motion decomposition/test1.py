# make sure the files from two folder have the same order
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
    return dist

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


filepath1 = r'registered_pose2/*.bvh'
filepath2 = r'registered_pose3/*.bvh'
filepath3 = r'registered_pose3Topose2/*.bvh'
fileList1 = glob.glob(filepath1)
fileList2 = glob.glob(filepath2)
fileList3 = glob.glob(filepath3)
mo_seg1 = []
mo_seg3 = []
for item in fileList1:
    mo_seg1.append(MotionSegment(item, type = 'file'))
for item in fileList3:
    mo_seg3.append(MotionSegment(item, type = 'file'))
mo_segs = mo_seg3 + mo_seg1
numFrames = mo_segs[0].numFrames
firstPrimitive = MorphableModel1(mo_segs)
index = np.random.randint(50,90)
print index
########################################
featureVector = getFeatureVector(mo_segs[index])
featureVector = np.array(featureVector)
firstPrimitive.projection(featureVector)
projectedVector = firstPrimitive.projectedFeatureVector
print projectedVector
#########################################
firstPrimitive.backprojection(projectedVector)
newfeature = firstPrimitive.backprojectedFeatureVector
newfeature = newfeature.tolist()
error = diff(featureVector, newfeature)
print error
priviousMotion = featureVectorToMotion(newfeature, numFrames)
oldmotion = MotionSegment(priviousMotion, fileList1[0], type = 'data')
oldmotion.display()
