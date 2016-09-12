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


def displayTwoMotion(mo1,mo2):
    # sequentially display two motions
    # make sure the first frame of motion 2 has the same orientation
    # and root position as the last frame of motion 1
    #root = mo1.data[-1][0:3]
    diff = np.array(mo2.data[0][0:3]) - np.array(mo1.data[-1][0:3])
    for item in mo2.data:
        temp = np.array(item[0:3]) - diff
        item[0:3] = temp.tolist()
    #mo2.display()
    dist,theta = distFrames(mo1.worldpos[mo1.numFrames-1], mo2.worldpos[0])
    print dist
    # rotate mo2 by angle theta
    angles = np.ones(mo2.numFrames)
    angles = angles*theta
    mo2.getMotion(angles)
    print mo1.worldpos[mo1.numFrames-1]['root']
    print mo2.worldpos[0]['root']
    #mo2.display()
    worldpos = {}
    worldpos = mo1.worldpos
    for index in mo2.worldpos:
        worldpos[index + mo1.numFrames] = mo2.worldpos[index]
    v = visualizer(mo1.hips, worldpos)
    v.animate_all()



    
filepath1 = r'registered_pose2/*.bvh'
filepath2 = r'registered_pose3/*.bvh'
filepath3 = r'registered_pose3Topose2/*.bvh'
fileList1 = glob.glob(filepath1)
fileList2 = glob.glob(filepath2)
fileList3 = glob.glob(filepath3)

##newlist1 = []
##newlist2 = []
##pattern = re.compile(r'\\')
##for item in fileList1:
##    filename = pattern.split(item)[-1]
##    #print filename
##    filename = 'registered_pose3\\' + filename
##    newlist1.append(item)
##    if filename in fileList2:
##        newlist2.append(filename)
#print fileList1
#print fileList2
mo_seg1 = []
mo_seg2 = []
mo_seg3 = []
for item in fileList1:
    mo_seg1.append(MotionSegment(item, type = 'file'))
for item in fileList2:
    mo_seg2.append(MotionSegment(item, type = 'file'))
for item in fileList3:
    mo_seg3.append(MotionSegment(item, type = 'file'))

#####
# generate a random index
num = len(fileList1)
#index = np.random.randint(0, num)
index1 = np.random.randint(0,num)
index2 = np.random.randint(0,num)
#print index
displayTwoMotion(mo_seg1[index1], mo_seg2[index2])
