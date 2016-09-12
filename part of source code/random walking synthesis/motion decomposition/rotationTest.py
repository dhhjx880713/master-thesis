import sys
include_lib = [r'../tools/motion segments/',r'../DTW/', ]
sys.path += include_lib
from MotionSegment import *
from distFrames import *
import glob


filepath2 = r'registered_pose2/*.bvh'
filepath3 = r'registered_pose3/*.bvh'
fileList2 = glob.glob(filepath2)
fileList3 = glob.glob(filepath3)
mo1 = MotionSegment(fileList2[0], type = 'file')
mo2 = MotionSegment(fileList3[0], type = 'file')
motion1 = {}
motion1[0] = mo1.worldpos[mo1.numFrames-1]
motion2 = {}
motion2[0] = mo2.worldpos[0]
dist, theta = distFrames(motion1, motion2)
print dist, theta
#############################################
# rotation and translation mo2
diff = np.array(mo2.data[0][0:3]) - np.array(mo1.data[-1][0:3])
for item in mo2.data:
    temp = np.array(item[0:3]) - diff
    item[0:3] = temp.tolist()
print mo1.data[-1][0:3]
print mo2.data[0][0:3]
#mo2.getMotion()
print mo1.worldpos[mo1.numFrames-1]['root']
print mo2.worldpos[0]['root']
rotationAxis = [0,1,0]
theta = np.pi
rotmat = rotation_matrix(theta, rotationAxis)
for i in range(mo2.numFrames):
    xrot = np.radians(mo2.data[i][3])
    yrot = np.radians(mo2.data[i][4])
    zrot = np.radians(mo2.data[i][5])
    root_rotation = euler_matrix(xrot, yrot, zrot, 'rxyz')
    root_orientation = np.dot(rotmat, root_rotation)
    euler_angles = euler_from_matrix(root_orientation, 'rxyz')
    mo2.data[i][3] = np.degrees(euler_angles[0])
    mo2.data[i][4] = np.degrees(euler_angles[1])
    mo2.data[i][5] = np.degrees(euler_angles[2])
mo2.getMotion()
mo2.display()
data = np.concatenate((mo1.data, mo2.data))
data = data.tolist()
newMO = MotionSegment(data, fileList2[0], type = 'data')
#newMO.display()
