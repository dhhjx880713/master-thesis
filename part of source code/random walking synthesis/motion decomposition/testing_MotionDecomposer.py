from MotionDecomposer import *
import glob

#filepath = r'../../mocap_data/walking/4m/*.bvh'
filepath = r'../motion decomposition/4m from door to wall/*.bvh'
fileList = glob.glob(filepath)
##print len(fileList)
##for item in fileList:
##    m = MotionDecomposer(item)
##    m.distanceBetweenFeet()
##    m.keyframeDetection()
##    m.segmentsGeneration()
##    writeDir = 'data\\'
##    m.writeBVH(writeDir)
mo = MotionSegment(fileList[2], type = 'file')
#mo.display()
m = MotionDecomposer(fileList[2])

#print m.motion.worldpos[0]
m.distanceBetweenFeet()
m.keyframeDetection()
m.segmentsGeneration()
#m.displaySegments()
#m.Ytranslation()
m.displayKeyframes()
writeDir = 'data\\'
m.writeBVH(writeDir)
