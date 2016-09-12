from MotionSegment import MotionSegment
import numpy as np
from transformations import *
import re
from scipy.ndimage.filters import gaussian_filter1d
class RegisteredMotionSegment(MotionSegment):
    
    def __init__(self, motion):
        self.hips = motion.hips
        self.data = motion.data[:]
        self.skeleton = motion.skeleton
        self.filepath = motion.filepath
        self.filename = motion.filename
        self.numFrames = motion.numFrames
        self.dt = motion.dt
        self.getMotion()

    def getWarppingIndex(self, warpIndex):
        # get warppedIndex and update motion data based on warp index
        self.warppingIndex = warpIndex
        warppedData = []
        for index in self.warppingIndex:
            warppedData.append(self.data[index][:])
        self.data = warppedData
        self.numFrames = len(self.data)

        

    def getRotationAngle(self, angle):
        # angle is in radian
        self.angle = angle

    def getAlignmentCurve(self, alignmentCurve):
        # alignmentCurve contains transition parameters: rotation angle of each frame
        self.alignmentCurve = alignmentCurve
        #print self.alignmentCurve
        

    def getOrigin(self, origin):
        # set the root position of reference motion as origin
        self.origin = np.array(origin)

    def smooth(self):
        # smooth the registered motion data
        num = len(self.warppingIndex)
        for i in range(num-2):
            if self.warppingIndex[i+1]+1 != self.warppingIndex[i+2]:

                temp = (np.array(self.data[i]) + np.array(self.data[i+2]))/2
                self.data[i+1] = temp.tolist()
        

    def transformation(self):
        # transform the original motion data using rotation angle
        
        origin = np.array(self.data[0][:3])
        
        rotationAxis = [0,1,0]
        #rotmat = rotation_matrix(self.angle, rotationAxis)
        point = np.ones(4)
        for i in range(self.numFrames):
            # normalize the root position, shift the root position of first frame to (0,0,0)
            newRoot = np.array(self.data[i][:3]) - origin
           
            point[:3] = newRoot
            rotmat = rotation_matrix(self.alignmentCurve[i], rotationAxis)
            newPoint = np.dot(rotmat, point.T)
            self.data[i][:3] = newPoint[:-1] + self.origin
            
            #self.data[i][:3] = newPoint[:-1]
            # x rotation angle
            xrot = np.radians(self.data[i][3])
            # y rotation angle (vertical axis)
            yrot = np.radians(self.data[i][4])
            # z rotation angle 
            zrot = np.radians(self.data[i][5])
        
            root_rotation = euler_matrix(xrot, yrot, zrot, 'rxyz')
            # rotate root orientation by angle
            root_orientation = np.dot(rotmat, root_rotation)
            # decompose the rotation matrix to euler angles
            euler_angles = euler_from_matrix(root_orientation, 'rxyz')
            # update root rotation anlges
            self.data[i][3] = np.degrees(euler_angles[0])
            self.data[i][4] = np.degrees(euler_angles[1])
            self.data[i][5] = np.degrees(euler_angles[2])
        
        #self.getMotion()

    def writeBVH(self):
        # output the registered motion data to BVH file
        pattern = re.compile(r'\\')
        filename = 'registered_' + pattern.split(self.filepath)[-1]
        outfile = open(filename, 'w')
        infile = file(self.filepath)
        s = infile.readline()
        tmp = s.strip()
        tmp = tmp.split()
        while tmp[0]!='MOTION' and tmp!= "":
            # output the current line
            outfile.write(s)
            s = infile.readline()
            tmp = s.strip()
            tmp = tmp.split()
        # write registered motion data, motion data is stored in self.data
        outfile.write('MOTION\n')
        outfile.write('Frames: '+str(self.numFrames)+'\n')
        outfile.write('Frame Time: ' + str(self.dt) + '\n')
        for frameIndex in range(self.numFrames):
            for value in self.data[frameIndex]:
                outfile.write('%s ' % str(value))
            outfile.write('\n')
        outfile.close()
        infile.close()
                
        

