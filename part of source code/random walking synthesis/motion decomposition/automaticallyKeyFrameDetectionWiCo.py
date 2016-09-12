###############################################################
# this script automatically extracts key frame indexes from WiCo motion
#  capture data
###############################################################
import sys
path = [r'../../../motion decomposition/', r'../../../tools/motion segments/']
#sys.path.append(path)
sys.path += path
from MotionDecomposer import *
import glob

testFolder = r'*.bvh'
features = ['L_Foot', 'R_Foot']
testFiles = glob.glob(testFolder)
outfile = file('key frame index.txt', 'wb')
for item in testFiles:
    decomposer = MotionDecomposer(item)
    decomposer.distanceBetweenFeet(features)
    decomposer.keyframeDetectionforAlternativeWalking(stepsize = 50, threshold = 10)
    outfile.write("file name : " + decomposer.filename +".bvh" + '\t')
    outfile.write("keyframes: ")
    for value in decomposer.keyframes:
        outfile.write('%s ' % str(value))
    outfile.write('\t')
    outfile.write("doubleStance: ")
    for value in decomposer.doubleStance:
        outfile.write('%s ' % str(value))
    outfile.write('\t')
    
    outfile.write("rightStance: ")
    for value in decomposer.rightStance:
        outfile.write('%s ' % str(value))
    outfile.write('\t')

    outfile.write("leftStance: ")
    for value in decomposer.leftStance:
        outfile.write('%s ' % str(value))
    outfile.write('\n')
outfile.close()
