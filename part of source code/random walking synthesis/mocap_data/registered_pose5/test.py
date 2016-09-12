import sys
path = r'../../../motion synthesis/tools/motion segments/'

sys.path.append(path)
from MotionSegment import *
import glob

def plotPath2D(worldpos):
    num = len(worldpos)
##    x = np.zeros(num)
##    y = np.zeros(num)
##    z = np.zeros(num)
##    for i in range(num):
##        x[i] = worldpos[i]['root'][0]
##        y[i] = worldpos[i]['root'][1]
##        z[i] = worldpos[i]['root'][2]
##    print z
    points = []
    for i in range(num):
        newpoint = worldpos[i]['root'][0:2]
        points.append(newpoint)
    
    fig = plt.figure()
    plt.plot(*np.transpose(points))
    plt.show()
    return points

filepath = r'*.bvh'
fileList = glob.glob(filepath)
#print fileList
mo_seg = []
for item in fileList:
    mo_seg.append(MotionSegment(item, type = 'file'))
for motion in mo_seg:
    #print motion.worldpos[0]['root']
    #plotPath2D(motion.worldpos)
    motion.display()
##m1 = MotionSegment(r'E:\visualizer\mocap_data\walking\pose2\pose2_1.bvh')
##m2 = MotionSegment(r'E:\visualizer\mocap_data\walking\pose2\pose2_8.bvh')
###registeredMotion = warpTwoMotion(m1, m2, DEBUG = 1)
##m1.display()
##m2.display()
##m2.displayOneFrame(10)
##m1.displayOneFrame(0)
