# make sure the files from two folder have the same order
import sys
include_lib = [r'../tools/motion segments/', r'../GMM/',r'../DTW/']
#sys.path.append(include_lib)
sys.path += include_lib
from MotionSegment import *
from MorphableModel import *
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
    motion1 = {}
    motion1[0] = mo1.worldpos[mo1.numFrames-1]
    motion2 = {}
    motion2[0] = mo2.worldpos[0]
    #dist,theta = distFrames(mo1.worldpos[mo1.numFrames-1], mo2.worldpos[0])
    dist,theta = distFrames(motion1, motion2)
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
    v = visualizer(mo1.hips, worldpos, mo1.skeleton)
    v.animate_all()



    
filepath1 = r'registered_correlated_pose2/*.bvh'
filepath2 = r'registered_correlated_pose3/*.bvh'
filepath3 = r'registered_correlated_pose3Topose2/*.bvh'
fileList1 = glob.glob(filepath1)
fileList2 = glob.glob(filepath2)
fileList3 = glob.glob(filepath3)

#print fileList1
#print fileList2
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
## #consturct morphable model
firstPrimitive = MorphableModel(filepath1)
firstPrimitive.createMorphableFunction()
nextPrimitive = MorphableModel(filepath2)
nextPrimitive.createMorphableFunction()
thirdPrimitive = MorphableModel(filepath3)
thirdPrimitive.createMorphableFunction()
# project data to new space
featureVector1 = []
featureVector2 = []
featureVector3 = []
for item in fileList1:
    m1 = MotionSegment(item, type = 'file')
    temp = getFeatureVector(m1)
    
    firstPrimitive.projection(temp)
    featureVector1.append(firstPrimitive.projectedFeatureVector)
for item in fileList2:
    m2 = MotionSegment(item, type = 'file')
    temp = getFeatureVector(m2)
    
    nextPrimitive.projection(temp)
    featureVector2.append(nextPrimitive.projectedFeatureVector)

for item in fileList3:
    m3 = MotionSegment(item, type = 'file')
    temp = getFeatureVector(m3)
    firstPrimitive.projection(temp)
    featureVector3.append(firstPrimitive.projectedFeatureVector)
frameNum3 = m3.numFrames
##print 'frameNum3: '
##print frameNum3
featureVector1 = np.array(featureVector1)
featureVector2 = np.array(featureVector2)
featureVector3 = np.array(featureVector3)
# write the featureVector1
##outfile = open('firstMotion.txt','w')
##m,n = featureVector1.shape
##for i in range(m):
##    for value in featureVector1[i,:]:
##        outfile.write('%s ' % str(value))
##    outfile.write('\n')
##outfile.close()
# testing
frameNum1 = m1.numFrames
frameNum2 = m2.numFrames
##print 'frameNum1:'
##print frameNum1
num = len(fileList3)
index = np.random.randint(0,num)
print index
firstPrimitive.backprojection(featureVector1[index])
newfeature = firstPrimitive.backprojectedFeatureVector
newfeature = newfeature.tolist()
priviousMotion = featureVectorToMotion(newfeature, frameNum1)
oldmotion = MotionSegment(priviousMotion, fileList1[0], type = 'data')
oldmotion.display()
##thirdPrimitive.backprojection(featureVector3[index])
##newfeature = thirdPrimitive.backprojectedFeatureVector
##newfeature = newfeature.tolist()
##priviousMotion = featureVectorToMotion(newfeature, frameNum3)
##oldmotion = MotionSegment(priviousMotion, fileList1[0], type = 'data')
##oldmotion.display()
##newMotion = featureVectorToMotion(newfeature, frameNum)
##motion = MotionSegment(newMotion, fileList1[0], type = 'data')
##motion.display()
# using Gaussian Process to model
# X is featureVector1, Y is featureVector2
GPs = []

#gp = GaussianProcess(corr = 'squared_exponential')

#y = np.random.randn(50)
#x = np.array([[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
######################################################################
# modeling the transition probability

m,n = featureVector2.shape

for featureIndex in range(n):
    gp = GaussianProcess(corr='squared_exponential', theta0=1e-2, thetaL=1e-4, thetaU=1e-1, \
                    random_start=100)
    gp.fit(featureVector1, featureVector2[:,featureIndex])
    GPs.append(gp)
#gp.fit(x, y)
##

nextMotion = np.zeros(n)
for i in range(n):
    nextMotion[i] = GPs[i].predict(featureVector3[index])
#print nextMotion
nextPrimitive.backprojection(nextMotion)
newFeature = nextPrimitive.backprojectedFeatureVector
newFeature = newFeature.tolist()

newMotion = featureVectorToMotion(newFeature, frameNum2)
motion = MotionSegment(newMotion, fileList1[0], type = 'data')
#motion.display()

displayTwoMotion(oldmotion, motion)
#########################################################################
# modeling two steps
