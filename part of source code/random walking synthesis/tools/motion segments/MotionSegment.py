import sys
include_path = r'../../MathAuxiliary/'
sys.path.append(include_path)

from skeleton import *
from visualizer import *
#from transformations import *
from transformations import rotation_matrix
import re
class MotionSegment:
##    def __init__(self, mocap_data, skeletonFile):
##        self.skeletonfile = skeletonFile
##        self.data = mocap_data
##        
##        self.getSkeleton()
##        
##        self.getMotion()
##        #self.display()
##
##    def __init__(self, bvhfile):
##        mybvh = readbvh(bvhfile)
##        mybvh.read()
##        self.filepath = bvhfile
##        self.hips = process_bvhnode(mybvh._root)
##        self.data = mybvh.keyframes
##        self.numFrames = mybvh.frames
##        self.dt = mybvh.dt
##        self.getMotion()
##        self.feature()

    def __init__(self, *args, **kwargs):
        #args -- tuple of anonymous arguments
        #kwargs -- dictionary of named arguments
        if 'type' in kwargs:
            type = kwargs.pop('type')
        else:
            raise ValueError('''Input data type must be specified. Either type = 'data' or type = 'file' ''')
        if type == 'file':
            mybvh = readbvh(args[0])
            mybvh.read()
            self.filepath = args[0]
           
            pattern = re.compile(r'/')
            self.filename = pattern.split(self.filepath)[-1]
            self.hips = process_bvhnode(mybvh._root)
            self.data = mybvh.keyframes
            self.numFrames = mybvh.frames
            self.dt = mybvh.dt
            self.skeleton = []
            self.getSkeleton(self.hips)
#            self.getMotion()
            
        elif type == 'data':
            self.data = args[0]
            self.filename = None
            self.skeletonfile = args[1]
            self.filepath = args[1]
            mybvh = readbvh(args[1])
            mybvh.read()
            self.dt = mybvh.dt
            self.hips = process_bvhnode(mybvh._root)
            self.numFrames = len(self.data)
            self.skeleton = []
            self.getSkeleton(self.hips)
            self.getMotion()
        else:
            raise ValueError('Unknown input data type!')
            
##    def getSkeleton(self):
##        mybvh = readbvh(self.skeletonfile)
##        mybvh.read()
##        self.hips = process_bvhnode(mybvh._root)
    def getSkeleton(self, joint):
        for child in joint.children:
            self.skeleton.append([joint.name, child.name])
        for child in joint.children:
            self.getSkeleton(child)
        

    def extractDataFromJoints(self, joint, index, oneFrameData={}):
        oneFrameData[joint.name] = [joint.worldpos[index][0],joint.worldpos[index][2],joint.worldpos[index][1]]
        if len(joint.children) != 0:
            for child in joint.children:
                oneFrameData = self.extractDataFromJoints(child,index, oneFrameData)
        return oneFrameData
    
    def getMotion(self, theta = []):
        
        # theta is in radians
        origin = np.array(self.data[0][:3])
        point = np.ones(4)
        rotationAxis = [0,1,0]
        self.worldpos = {}
        for i in range(self.numFrames):
            
            if theta != []:
                
                
                root = np.array(self.data[i][:3]) - origin
                point[:-1] = root
                rotmat = rotation_matrix(theta[i], rotationAxis)
                newRoot = np.dot(rotmat, point.T)
                self.data[i][:3] = newRoot[:-1] + origin
                
                process_bvhkeyframe(self.data[i],self.hips, i, theta[i], 0)
                #process_bvhkeyframe(self.data[i],self.hips, i, 0)
                
            else:
                
                process_bvhkeyframe(self.data[i],self.hips, i, 0)
                
            
            oneFrameData = self.extractDataFromJoints(self.hips, i)
            self.worldpos[i] = oneFrameData.copy()
    
    def getFrame(self, frameIndex):
        process_bvhkeyframe(self.data[frameIndex],self.hips, frameIndex, 0, DEBUG=1)
        oneFrameData = self.extractDataFromJoints(self.hips, frameIndex)
        

    def display(self):
        v = visualizer(self.hips,self.worldpos, self.skeleton, self.filename)
        v.animate_all()

    def feature(self):
        # use the angle of two legs as feature of each frame
        feature1 = "left_hip"
        feature2 = "right_hip"
        feature3 = "left_knee"
        feature4 = "right_knee"
        self.features = []
        for i in range(self.numFrames):
            point1 = np.array(self.worldpos[i][feature1])
            point2 = np.array(self.worldpos[i][feature2])
            point3 = np.array(self.worldpos[i][feature3])
            point4 = np.array(self.worldpos[i][feature4])
            vec1 = point1 - point3
            
            vec2 = point2 - point4
            
            self.features.append(np.dot(vec1,vec2)/(np.linalg.norm(vec1)* np.linalg.norm(vec2)))

    def displayOneFrame(self, frame_num):
        v = visualizer(self.hips,self.worldpos, self.skeleton)
        v.plotOneFrame(frame_num)
    def animate_scene(self,animation_range):
        v = visualizer(self.hips,self.worldpos, self.skeleton)
        v.animate_scene(animation_range)
    def displayFrames(self,frames, color = 'blue'):
        v = visualizer(self.hips, self.worldpos, self.skeleton)
        v.plotFrames(frames, color)
    def writeMP4(self):
        v = visualizer(self.hips, self.worldpos, self.skeleton)
        v.writeMP4()
    def displayAllFrames(self, color = 'blue'):
        frame_num = np.arange(0, self.numFrames,2)
        self.displayFrames(frame_num, color)
        
    def writeBVH(self):
        infile = file(self.filepath)
        filename = 'new motion.bvh'
        outfile = open(filename, 'w')
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

def test():
    m = MotionSegment(r'C:\Users\hadu01\MG++\repo\src\6 - Motion synthesis\lib\skeleton.bvh', type = 'file')
#    print m.worldpos[46]['Bip01_L_Toe0']
    m.getFrame(46)
if __name__ == "__main__":
    test()


##m = MotionSegment(r'1.bvh')
##m.displayOneFrame(20)
