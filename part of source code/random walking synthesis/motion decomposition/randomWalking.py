import sys
include_lib = [r'../tools/motion segments/',r'../DTW/', r'../GMM']
sys.path += include_lib
from MotionSegment import *
from distFrames import *

import glob
import re
class randomWalking():
    
    def __init__(self, num_steps, fileLists):
        self.numSteps = num_steps
        self.fileLists = fileLists
        self.loadData()
        print 'load data is finished'
        self.pathPlanning()
        self.stepSynthesisFromTrainingData()
        self.saveBVH()
        self.connectMotions()
        
        
    def loadData(self):
        # load the motion capture data
        
        self.mo_segs = {}
        num_fileList = len(self.fileLists)
        for i in range(num_fileList):
            self.mo_segs[i] = []
            for item in self.fileLists[i]:
                mo_seg = MotionSegment(item, type = 'file')
                self.mo_segs[i].append(mo_seg)

    def pathPlanning(self):
        # plan the plan based on motion graph
        midsteps = self.numSteps - 2
        firstStep = np.random.randint(2)
        previousStep = firstStep
        self.path = [previousStep, ]
        while midsteps!= 0:
            nextStep = 0
            if previousStep == 0:
                nextStep = 2
            elif previousStep == 1:
                nextStep = 3
            elif previousStep == 2:
                nextStep = 3
            else:
                nextStep = 2
            self.path.append(nextStep)
            midsteps = midsteps - 1
            previousStep = nextStep
        if previousStep == 0:
            lastStep = 5
        elif previousStep == 1:
            lastStep = 4
        elif previousStep == 2:
            lastStep = 4
        else:
            lastStep = 5
        self.path.append(lastStep)
        
    def stepSynthesisFromTrainingData(self):
        # generate first step

        self.motionList = []  # a list of motion segments
        index = np.random.randint(len(self.mo_segs[self.path[0]]))
        #previousStep = self.mo_segs[self.path[0]][index]
        self.motionList.append(self.mo_segs[self.path[0]][index])
        for i in range(len(self.path)-1):
            count = 0
            dist = 50000
            while dist > 9000 and count != 50:
                index = np.random.randint(len(self.mo_segs[self.path[i+1]]))
                newMotion = self.mo_segs[self.path[i+1]][index]
                motion1 = {}
                motion2 = {}
                motion1[0] = self.motionList[-1].worldpos[self.motionList[-1].numFrames-1]
                motion2[0] = newMotion.worldpos[0]
                dist, theta = distFrames(motion1, motion2, weighted = 1)
                print dist
                count = count + 1
            print "the number of count is: " + str(count)
            if count == 0:
                self.stepSynthesisFromTrainingData()
            self.motionList.append(newMotion)
            
    def saveBVH(self):
        num = len(self.motionList)
        self.data = self.motionList[0].data
        rotationAxis = [0,1,0]
        for index in range(num-1):
            currentMo = self.motionList[index]
            nextMo = self.motionList[index+1]
            diff = np.array(nextMo.data[0][0:3]) - np.array(currentMo.data[-1][0:3])
            for item in nextMo.data:
                temp = np.array(item[0:3]) - diff
                item[0:3] = temp.tolist()
            motion1 = {}
            motion1[0] = currentMo.worldpos[currentMo.numFrames-1]
            motion2 = {}
            motion2[0] = nextMo.worldpos[0]
            dist,theta = distFrames(motion1, motion2)
            print dist
            angles = np.ones(nextMo.numFrames)
            angles = theta*angles
            nextMo.getMotion(angles)
            self.motionList[index+1].worldpos = nextMo.worldpos
            # rotate the root orientation in nextMo by theta
            rotmat = rotation_matrix(theta, rotationAxis)
            for i in range(nextMo.numFrames):
                xrot = np.radians(nextMo.data[i][3])
                yrot = np.radians(nextMo.data[i][4])
                zrot = np.radians(nextMo.data[i][5])
                root_rotation = euler_matrix(xrot, yrot, zrot, 'rxyz')
                root_orientation = np.dot(rotmat, root_rotation)
                euler_angles = euler_from_matrix(root_orientation, 'rxyz')
                # update root orientation
                nextMo.data[i][3] = np.degrees(euler_angles[0])
                nextMo.data[i][4] = np.degrees(euler_angles[1])
                nextMo.data[i][5] = np.degrees(euler_angles[2])

            self.data = np.concatenate((self.data, nextMo.data),axis = 0)

        self.numFrames = len(self.data)
        self.data = self.data.tolist()
        newMoSeg = MotionSegment(self.data, self.fileLists[0][0], type = 'data')
        #newMoSeg.display()
        ###################################
        ## output the concatenated motion
        pattern = re.compile(r'\\')
        filename = 'randomWalk' + str(self.numSteps) + 'steps.bvh'
        outfile = open(filename, 'wb')
        infile = file(self.fileLists[0][0])
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
        outfile.write('Frame Time: ' + str(self.motionList[0].dt) + '\n')
        for frameIndex in range(self.numFrames):
            for value in self.data[frameIndex]:
                outfile.write('%s ' % str(value))
            outfile.write('\n')
        outfile.close()
        infile.close()
        newMoSeg.display()

    def connectMotions(self):
        print "motion list: "
        for motion in self.motionList:
            print motion.filename
        # connecting the motions in motion list in a rotation and translation invariant way
        num = len(self.motionList)
        worldpos = self.motionList[0].worldpos
        for index in range(num-1):
            currentMo = self.motionList[index]
            nextMo = self.motionList[index+1]
            diff = np.array(nextMo.data[0][0:3]) - np.array(currentMo.data[-1][0:3])
            for item in nextMo.data:
                temp = np.array(item[0:3]) - diff
                item[0:3] = temp.tolist()
            motion1 = {}
            motion1[0] = currentMo.worldpos[currentMo.numFrames-1]
            motion2 = {}
            motion2[0] = nextMo.worldpos[0]
            dist,theta = distFrames(motion1, motion2, weighted = 1)
            print "distance is: " + str(dist)
            print "angle is: " + str(theta)
            # rotate the nextMo by angle theta
            angles = np.ones(nextMo.numFrames)
            angles = theta*angles
            nextMo.getMotion(angles)
            self.motionList[index+1].worldpos = nextMo.worldpos
            # put the pose data into worldpos
            offset = len(worldpos)
            for i in nextMo.worldpos:
                worldpos[offset + i] = nextMo.worldpos[i]
        #points = plotPath2D(worldpos)
##        dist, orientations = distance(points)
##        #print dist
##        # generate new motion segment
##        # get the inital data
##        data = []
##        for index in range(num):
##            data += motionList[index].data
##    
##        num = len(data)
##        #print num
##        for i in range(num):
##            #data[i][3:6] = data[0][3:6]
##            data[i][1] = data[0][1]
##            if i == 0:
##                data[i][0] = -data[i][6]
##                data[i][2] = -data[i][8]
##            else:
##                data[i][0] = data[0][0]
##                data[i][2] = data[i-1][2] - dist[i-1]
##        newMotion = MotionSegment(data, fileList1[0],type = 'data')
##        angles = np.ones(num)
##        theta = -35.0/180.0*np.pi
##        angles = angles*theta
##        newMotion.getMotion(angles)
##    print 'called'
##    print dist
##    print -orientations
##    newMotion.getMotion(-orientations)
    #newMotion.display()
        v = visualizer(self.motionList[0].hips, worldpos, self.motionList[0].skeleton)
        v.animate_all()
        #return newMotion
#############################################################
# testing
##filepath0 = r'registered_pose0/*.bvh'
##filepath1 = r'registered_pose1/*.bvh'
##filepath2 = r'registered_pose2/*.bvh'
##filepath3 = r'registered_pose3/*.bvh'
##filepath4 = r'registered_pose4/*.bvh'
##filepath5 = r'registered_pose5/*.bvh'
filepath0 = r'pose0/*.bvh'
filepath1 = r'pose1/*.bvh'
filepath2 = r'pose2/*.bvh'
filepath3 = r'pose3/*.bvh'
filepath4 = r'pose4/*.bvh'
filepath5 = r'pose5/*.bvh'

fileList0 = glob.glob(filepath0)
fileList1 = glob.glob(filepath1)
fileList2 = glob.glob(filepath2)
fileList3 = glob.glob(filepath3)
fileList4 = glob.glob(filepath4)
fileList5 = glob.glob(filepath5)
fileLists = [fileList0, fileList1, fileList2, fileList3, fileList4, fileList5]
try:
    input_num = int(raw_input('Please input the steps number : '))
except ValueError:
    print "Not a number"
w = randomWalking(input_num, fileLists)
