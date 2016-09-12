#################################################################
# automatic motion decomposition for high quality motion data
#################################################################
from MotionDecomposer import *
import glob
#testfile = r'../mocap_data/walking/high quality data/4kmh.bvh'
#testfile = r'../mocap_data/walking/high quality data/6kmh001.bvh'
testFolder = r'../mocap_data/walking/high quality data/'
fileList = glob.glob(testFolder)
for item in fileList:
    decomposer = MotionDecomposer(item)
    features = ['L_Foot', 'R_Foot']
    decomposer.distanceBetweenFeet(features, DEBUG = 0)
    decomposer.keyframeDetectionforAlternativeWalking(stepsize = 70, threshold = 10)
    #decomposer.displayKeyframes()
    decomposer.segmentsGeneration()
    writeDir = 'data\\'
    decomposer.writeBVH(writeDir)

