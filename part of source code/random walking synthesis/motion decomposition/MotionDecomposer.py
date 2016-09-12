import sys

path = r'../tools/motion segments/'
sys.path.append(path)
from MotionSegment import *
from scipy.ndimage.filters import gaussian_filter1d
from visualizer import *
import re

class motionVisualizer(visualizer):
    def __init__(self, motionSegment, fig):
        self.skeleton = []
        self.hips = motionSegment.hips
        self.worldpos = motionSegment.worldpos
        self.fig = fig
        self.ax = p3.Axes3D(self.fig)
        self.frameLen = motionSegment.numFrames
        self.origin = motionSegment.worldpos[0][self.hips.name]
        self.getSkeleton(self.hips)
        self.ytrans = 0  # for BVH file, y-axis is the vertical axis
        self.boundary()
##        self.ax.set_xlim3d(-4000+self.origin[0], 4000 + self.origin[0])
##        self.ax.set_ylim3d(-3000+self.origin[1], 3000+self.origin[1])
##        self.ax.set_zlim3d(-1000+self.origin[2], 3000+self.origin[2])
        offset = (self.xmax - self.xmin) - (self.ymax - self.ymin)
        self.ax.set_xlim3d(self.xmin, self.xmax)
        #self.ax.set_ylim3d(self.ymin - offset/2, self.ymax + offset/2)
        self.ax.set_ylim3d(self.ymin - offset, self.ymax + offset)
        self.ax.set_zlim3d(self.zmin, self.zmax*2)
        

    def plotframesRange(self, plotRange, color):
        for index in range(plotRange[0],plotRange[1],3):
            for item in self.skeleton:
                tmp = [self.worldpos[index][item[0]], self.worldpos[index][item[1]]]
                tmp = np.transpose(np.array(tmp))
                self.ax.plot(*tmp, color = color)
                
    def getSkeleton(self, joint):
        for child in joint.children:
            self.skeleton.append([joint.name, child.name])
        for child in joint.children:
            self.getSkeleton(child)
        

    def plotframes(self, frameIndex, color = 'b'):
        if frameIndex != []:
            
            for index in frameIndex:
                for item in self.skeleton:
                    tmp = [self.worldpos[index][item[0]], self.worldpos[index][item[1]]]
                    tmp = np.transpose(np.array(tmp))
                    self.ax.plot(*tmp, color = color)
        else:
            pass

class MotionDecomposer:
    # decompose the input bvh file into motion segments
    def __init__(self, filename):
        
        #self.filename = filename
        # extract file name
        self.filepath = filename
        pattern = re.compile(r'/')
        temp = pattern.split(filename)[-1]
        pattern = re.compile(r'\\')
        temp = pattern.split(temp)[-1]
        self.filename = re.sub('.bvh', '', temp)
        #print self.filename
        self.motion = MotionSegment(filename, type = 'file')
        
    def distanceBetweenFeet(self, features = ['right_toes', 'left_toes'], DEBUG = 0):
        self.features = features
        if self.motion.worldpos[0][self.features[0]][-1] > 0 and self.motion.worldpos[0][self.features[1]][-1] > 0:
            self.ytrans =  np.min([self.motion.worldpos[0][self.features[0]][-1], self.motion.worldpos[0][self.features[1]][-1]])
        else: 
            self.ytrans = -np.min([self.motion.worldpos[0][self.features[0]][-1], self.motion.worldpos[0][self.features[1]][-1]])
        
            
        self.feetDistance = np.zeros(self.motion.numFrames)
        for i in range(self.motion.numFrames):
            vec1 = np.array(self.motion.worldpos[i][self.features[0]][:-1])
            vec2 = np.array(self.motion.worldpos[i][self.features[1]][:-1])
            self.feetDistance[i] = np.linalg.norm( vec1 - vec2)
        sigma = 2
        self.feetDistance = gaussian_filter1d(self.feetDistance, sigma)
        if DEBUG == 1:
            fig = plt.figure()
            plt.plot(self.feetDistance)
            plt.xlabel('Frame No.')
            plt.ylabel('Absolute distance')
            plt.show()

    def keyframeDetection(self , stepsize = 30):
        # detect local maximum from distance bewteen two feet
        # smooth the feature data
        
        
        self.local_maximum(stepsize)
        # classify local maximum point
        self.doubleStance = [0, self.motion.numFrames-1]
        self.leftStance = []
        self.rightStance = []
        for index in self.localMax:
            origin = np.array(self.motion.worldpos[0][self.motion.hips.name][:-1])
            vec0 = np.array(self.motion.worldpos[index][self.features[0]][:-1])
            vec1 = np.array(self.motion.worldpos[index][self.features[1]][:-1])
            dist1 = np.linalg.norm(vec0 - origin)
            dist2 = np.linalg.norm(vec1 - origin)
            if dist1 > dist2:
                if 'right' in self.features[0]:
                    self.rightStance.append(index)
                else:
                    self.leftStance.append(index)
            else:
                if 'right' in self.features[0]:
                    self.leftStance.append(index)
                else:
                    self.rightStance.append(index)
        keyframes = self.doubleStance + self.leftStance + self.rightStance

        keyframes.sort()
        self.keyframes = keyframes

    def keyframeDetectionforAlternativeWalking(self , stepsize = 30, threshold = 15):
        # detect local maximum from distance bewteen two feet
        # smooth the feature data
        
        
        self.local_maximum(stepsize, threshold)
        # classify local maximum point
        # self.doubleStance = [0, self.motion.numFrames-1]
        self.doubleStance = []
        self.leftStance = []
        self.rightStance = []
##        for index in self.localMax:
##            origin = np.array(self.motion.worldpos[0][self.motion.hips.name][:-1])
##            vec0 = np.array(self.motion.worldpos[index][self.features[0]][:-1])
##            vec1 = np.array(self.motion.worldpos[index][self.features[1]][:-1])
##            dist1 = np.linalg.norm(vec0 - origin)
##            dist2 = np.linalg.norm(vec1 - origin)
##            if dist1 > dist2:
##                if 'right' or 'R' or 'r' in self.features[0]:
##                    self.rightStance.append(index)
##                else:
##                    self.leftStance.append(index)
##            else:
##                if 'right' or 'R' or 'r' in self.features[0]:
##                    self.leftStance.append(index)
##                else:
##                    self.rightStance.append(index)
        for index in self.localMax:
            if self.motion.worldpos[index][self.features[0]][1] < self.motion.worldpos[index][self.features[1]][1]:
                if 'right' or 'R' or 'r' in self.features[0]:
                    self.rightStance.append(index)
                else:
                    self.leftStance.append(index)
            else:
                if 'right' or 'R' or 'r' in self.features[0]:
                    self.leftStance.append(index)
                else:
                    self.rightStance.append(index)
##        print 'left stance index'
##        print self.leftStance
##        print 'right stance index'
##        print self.rightStance

        self.keyframes = self.leftStance + self.rightStance
        self.keyframes.sort()
        #print self.localMax
##        fig = plt.figure()
##        plt.plot(self.feetDistance)
##       
##        plt.show()
        ##
    def segmentsGeneration(self):
        # generate motion segments according to key frames
        # label: pose0: right first starting    from double stance to right stance
        #        pose1: left first starting     from double stance to left stance
        #        pose2: right first walking     from right stance to left stance
        #        pose3: left first walking      from left stance to right stance
        #        pose4: left first ending       from left stance to double stance
        #        pose5: right first ending      from right stance to double stance
        self.pose = {}
        # initialize self.pose
        for i in range(6):
            self.pose[i] = []
        # color for different pose
        self.colors = ['blue', 'green', 'black', 'red', 'orange', 'purple']
        # first, sort key frames
        keyframes = self.doubleStance + self.leftStance + self.rightStance

        keyframes.sort()
##        print "key frame index"
##        print keyframes
##        print " total number of poses: " + str(len(keyframes) - 1)
        count = 0
        for i in range(len(keyframes) - 1):
            segment = [keyframes[i], keyframes[i+1]]
            if keyframes[i] in self.doubleStance and keyframes[i+1] in self.rightStance:
                # pose 0
                self.pose[0].append(segment)
            elif keyframes[i] in self.doubleStance and keyframes[i+1] in self.leftStance:
                # pose 1
                self.pose[1].append(segment)
            elif keyframes[i] in self.rightStance and keyframes[i+1] in self.leftStance:
                # pose 2
                self.pose[2].append(segment)
            elif keyframes[i] in self.leftStance and keyframes[i+1] in self.rightStance:
                # pose 3
                self.pose[3].append(segment)
            elif keyframes[i] in self.leftStance and keyframes[i+1] in self.doubleStance:
                # pose 4
                self.pose[4].append(segment)
            elif keyframes[i] in self.rightStance and keyframes[i+1] in self.doubleStance:
                # pose 5
                self.pose[5].append(segment)
            else:
                print "Unknown pose"
                print segment
                count = count + 1
        #print "total number of unknown pose: " + str(count)

                
    def local_maximum(self, stepsize, threshold = 200):
        #print 'threshold is :' + str(threshold)
        # mirror the signal on the boundary with size = stepsize/2
        k = int(np.floor(stepsize/2))
        extendedSignal = np.zeros(self.motion.numFrames + 2*k)
        for i in range(self.motion.numFrames):
            extendedSignal[i+k] = self.feetDistance[i]
        for i in range(k):
            extendedSignal[i] = self.feetDistance[0]
            extendedSignal[-1-i] = self.feetDistance[-1]
        localMax = []
        for j in range(self.motion.numFrames):
            searchArea = extendedSignal[j: j+stepsize +1]
            
            maximum = np.max(searchArea)
            if maximum == self.feetDistance[j]:
                localMax.append(j)
        #print localMax
        # local_maximum does not contain the first and last elements of the signal
        if localMax[0] == 0:
            del localMax[0]
        if localMax[-1] == self.motion.numFrames - 1:
            del localMax[-1]
        
        # remove the local maxima if the difference bewteen this maxima and the former minima is too small
        #threshold = np.min([self.feetDistance[0],self.feetDistance[-1]])
        #threshold = 10
        #threshold = 200
        tmp = []
        for i in range(len(localMax)):
            
            if i == 0:
                localMin = np.min(self.feetDistance[0:localMax[i]])
                if localMin == self.feetDistance[0]:
                    localMin = 0
                
            else:
                localMin = np.min(self.feetDistance[localMax[i-1]:localMax[i]])
            if self.feetDistance[localMax[i]] - localMin > threshold:
                tmp.append(localMax[i])
        self.localMax = tmp
##        print "local Max"
##        print self.localMax

    def displayInputMotion(self):
        fig = plt.figure()
        v = motionVisualizer(self.motion, fig)
        frameIndex = range(0, self.motion.numFrames, 3)
        v.plotframes(frameIndex, 'b')
        plt.title('4 meters walking')
        plt.show()
        
    def displaySegments(self):
        
        fig = plt.figure()
        v = motionVisualizer(self.motion,fig)
        for index in self.pose:
            if self.pose[index]:
                for segment in self.pose[index]:
                    v.plotframesRange(segment, self.colors[index])
        plt.title('segmented motion')
        plt.show()

    def displayKeyframes(self):
        fig = plt.figure()
        v = motionVisualizer(self.motion, fig)
        v.plotframes(self.doubleStance, 'b')
        v.plotframes(self.rightStance, 'r')
        v.plotframes(self.leftStance, 'g')
        plt.title('key frames')
        plt.show()
        
    def Ytranslation(self):
        # for BVH file, y-axis is the vertical axis
        for i in range(self.motion.numFrames):
            self.motion.data[i][1] += self.ytrans
            
    def writeBVH(self, writeDir):
        # export the result of motion segmentation as bvh files, each file contains one segment
        
        for index in self.pose:
            if self.pose[index]:
                for segment in self.pose[index]:
                    filename =writeDir+ self.filename + '_' + str(segment[0]) + 'to' + str(segment[1])+ '_' + str(index) + '.bvh'
                    print filename
                    # create an output file
                    outfile = open(filename, 'w')
                    # write the hierarchy of bvh file, just copy the input file
                    # read one line from the input file
                    infile = file(self.filepath)
                    s = infile.readline()
                    tmp = s.strip()   # remove the white space of a line
                    tmp = tmp.split()  # split the line by white space
                    while tmp[0]!= 'MOTION' and tmp!= "":
                        # output the current line
                        outfile.write(s)
                        s = infile.readline()
                        tmp = s.strip()
                        tmp = tmp.split()
                    # write the motion data, motion data is stored in self.motion.data
                    outfile.write('MOTION\n')
                    num_frames = segment[1] - segment[0] + 1
                    outfile.write('Frames: '+str(num_frames) + '\n')
                    outfile.write('Frame Time: '+ str(self.motion.dt)+'\n')
                    for frameindex in range(segment[0], segment[1]+1):
                        for value in self.motion.data[frameindex]:
                            outfile.write('%s ' % str(value))
                        outfile.write('\n')
                    outfile.close()
                    infile.close()
        
                    

                
