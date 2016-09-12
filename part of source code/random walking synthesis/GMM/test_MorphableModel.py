import sys
include_path = r'../tools/motion segments/'
sys.path.append(include_path)
from MorphableModel import *
from RegisteredMotionSegment import *


#######################################################
def getFeatureVector(motionSegment):
    featureVector = []
    for frame in motionSegment.data:
        #featureVector = featureVector + frame
        #selectedFeature = frame[6:]
        selectedFeature = frame[:]
        featureVector = featureVector + selectedFeature
    return featureVector
filepath0 = r'../mocap_data/registered_pose0/*.bvh'
filepath1 = r'../mocap_data/registered_pose1/*.bvh'
filepath2 = r'../mocap_data/registered_pose2/*.bvh'
filepath3 = r'../mocap_data/registered_pose3/*.bvh'
filepath4 = r'../mocap_data/registered_pose4/*.bvh'
filepath5 = r'../mocap_data/registered_pose5/*.bvh'
file1 = r'../mocap_data/registered_pose2/registered_2_141to176_2.bvh'
##fileList0 = glob.glob(filepath0)
##fileList1 = glob.glob(filepath1)
##fileList2 = glob.glob(filepath2)
##fileList3 = glob.glob(filepath3)
fileList = glob.glob(filepath3)
mo_segs = []
for item in fileList:
    mo_seg = MotionSegment(item, type = 'file')
    mo_segs.append(mo_seg)

def featureVectorToMotion(featureVector,frameNum):
    # the input featureVector should be an array list
    featureVector = featureVector.tolist()
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
# create weights
M = MorphableModel(mo_segs, type = 'MotionSegments')

numFrames = M.mo_segments[0].numFrames
lenFeatures = len(M.mo_segments[0].data[0])

    
M.createWeightedMorphableFunction()
#print M.num
## # display an original data
##m1 = MotionSegment(file1, type = 'file')
#m1.display()
################################################################
##featureVector = getFeatureVector(m1)
##M.projection(featureVector)
##compactFeatureVector = M.projectedFeatureVector
##M.backprojection(compactFeatureVector)
##newFeature = M.backprojectedFeatureVector
##newFeature = newFeature.tolist()
##error = diff(featureVector, newFeature)
###print error
##frameNum = m1.numFrames
##newMotion = featureVectorToMotion(newFeature,frameNum)
##m2 = MotionSegment(newMotion, file1, type='data')
#m2.display()
##################################################################
# generate a random motion
frameNum = M.mo_segments[0].numFrames
M.sampleWeightedPCA()
newMotion = M.newMotion
MotionData = featureVectorToMotion(newMotion, frameNum)
m3 = MotionSegment(MotionData, fileList[0], type='data')
#registered_mo = RegisteredMotionSegment(m3)
#registered_mo.setFileName('noisyMotion.bvh')
#registered_mo.writeBVH('synthesizedMotion.bvh')
m3.display()
