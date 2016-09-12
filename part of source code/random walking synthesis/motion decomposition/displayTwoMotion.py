import sys
include_path = r'../DTW/'
sys.path.append(include_path)
from DTW import *

def displayTwoMotion1(mo1,mo2):
    worldpos = {}
    worldpos = mo1.worldpos
    for index in mo2.worldpos:
        worldpos[index + mo1.numFrames] = mo2.worldpos[index]
    v = visualizer(mo1.hips, worldpos, mo1.skeleton)
    v.animate_all()

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

##file1 = r'../motion decomposition/registered_correlated_pose2/registered_1.bvh'
##file2 = r'../motion decomposition/registered_correlated_pose3/registered_1.bvh'
##mo1 = MotionSegment(file1, type = 'file')
##mo2 = MotionSegment(file2, type = 'file')
fileFolder = r'pose3/*.bvh'
fileFolder1 = r'pose2/*.bvh'
fileList = glob.glob(fileFolder)
fileList1 = glob.glob(fileFolder1)
##mo = MotionSegment(fileList[17], type = 'file')
##mo.displayAllFrames(color = 'black')
mo1 = MotionSegment(fileList1[5], type = 'file')
mo1.displayAllFrames(color = 'r')
##file1 = r'pose2/5.bvh'
##file2 = r'pose3/1.bvh'
##ref1 = r'pose2/13.bvh'
##ref2 = r'pose3/34.bvh'
##mo1 = MotionSegment(file1, type = 'file')
##mo2 = MotionSegment(file2, type = 'file')
#mo2.display()
#displayTwoMotion1(mo1,mo2)
##ref_mo1 = MotionSegment(ref1, type = 'file')
##ref_mo2 = MotionSegment(ref2, type = 'file')
##
##registeredMo1 = warpTwoMotion(mo1,ref_mo1)
##registeredMo2 = warpTwoMotion(mo2,ref_mo2)

##
##angles = np.ones(mo2.numFrames)
##angles = angles*np.pi
##mo2.getMotion(angles)
##mo2.display()
##re_mo2 = RegisteredMotionSegment(mo2)
###angle = np.radians(180)
##angle = 180
##angles = np.ones(mo2.numFrames)
##angles = angles*angle
##mo2.getMotion(angles)
#mo2.display()
#re_mo2.getRotationAngle(angle)
##re_mo2.getOrigin(re_mo2.worldpos[0]['root'])
##re_mo2.getAlignmentCurve(angles)
##re_mo2.smoothAlignmentCurve()
##re_mo2.transformation()
##re_mo2.display()
####registeredMo2.getMotion(angles)
##registeredMo2.display()
#registeredMo2.display()
##registeredMo2.display()
##displayTwoMotion(mo1, mo2)
