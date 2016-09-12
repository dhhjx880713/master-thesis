########################################################################
# this script displays a 4m walking motion, then extract pre-defined motion 
########################################################################
from MotionDecomposer import *
import glob
##testfile = r'..\mocap_data\walking\straight line walking one way\1.bvh'
##testMotion = MotionSegment(testfile, type = 'file')
##fig = plt.figure()
##v = motionVisualizer(testMotion, fig)
##color = 'b'
##frameIndex = np.arange(testMotion.numFrames)
##v.plotframes(frameIndex, color)

##testFolder = r'..\mocap_data\walking\straight line walking one way\*.bvh'
##testFileList = glob.glob(testFolder)
##decomposer = MotionDecomposer(testFileList[2])
testFile = r'..\mocap_data\walking\straight line walking one way\110.bvh'
decomposer = MotionDecomposer(testFile)

decomposer.distanceBetweenFeet(DEBUG = 1)
decomposer.keyframeDetection(stepsize = 30)
decomposer.segmentsGeneration()
decomposer.displayInputMotion()
decomposer.displayKeyframes()
decomposer.displaySegments()
