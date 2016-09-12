import sys
include_lib = [r'../tools/motion segments/', r'../DTW/']
sys.path += include_lib
from MotionSegment import *
from DTW import *
testfile = r'pose2/148_75to115_2.bvh'
testfile2 = r'pose2/165_121to153_2.bvh'
#testfile = r'registered_pose2/registered_104_51to137_2.bvh'
refFile = r'pose2/51_34to66_2.bvh'
mo = MotionSegment(testfile, type = 'file')
mo.display()
mo2 = MotionSegment(testfile2, type = 'file')
##mo.display()
refMo = MotionSegment(refFile, type = 'file')
#mo.display()
registeredMotion = warpTwoMotion(mo2, refMo, DEBUG = 1)
