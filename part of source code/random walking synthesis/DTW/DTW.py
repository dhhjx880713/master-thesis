import sys
include_path = r'../tools/motion segments/'
sys.path.append(include_path)
from MotionSegment import *
from distMotions import *
import glob

class DTW():
    
    def __init__(self,directoryWithSegmentedBVHFiles):
        # the input for constructor is a file folder which contains a set of bvh file
        self.fileList = glob.glob(directoryWithSegmentedBVHFiles)
        
        self.numMotion = 0
        self.getMotionData()
        self.refIdx = 0

    def getMotionData(self):
        # create a list of unregistered motion segments
        self.motionData = []
        for f in self.fileList:
            
            self.motionData.append(MotionSegment(f, type = 'file'))
        self.numMotion = len(self.motionData)

        
    def setReferenceMotion(self, filename):    
        for i in range(self.numMotion):
            if filename in self.fileList[i]:
                self.refIdx = i
                break
        if i == self.numMotion-1:
            raise ValueError('reference file is not found in the folder!')
        
    def findReferenceMotion(self):
        # reference motion is defined as the motion which minimizes average distance to the other motions
        dist = np.zeros(self.numMotion)
        for i in range(self.numMotion):
            print 'motion_'+str(i)
            averageDist = 0
            for j in range(self.numMotion):
                if j!=i:
                    print j
                # register motion j to motion i
                    registeredMotion = warpTwoMotion(self.motionData[j], self.motionData[i])
                    tmp = distMotions(registeredMotion, self.motionData[i])
                    #print tmp
                    averageDist += tmp
                
            dist[i] = averageDist/self.numMotion
            #print dist[i]
        # find the index of smallest dist in dist list
        self.refIdx = min(xrange(self.numMotion), key = dist.__getitem__)

    def writeReferenceMotion(self):
        # copy the original data and export it out
        refMotion = self.motionData[self.refIdx]
        registeredRefMotion = RegisteredMotionSegment(refMotion)
        registeredRefMotion.writeBVH()
        

    def warpMotionSegmentsToReferenceMotion(self):
        #
        for i in range(self.numMotion):
            if i!= self.refIdx:
                # warp current motion to reference motion
                registeredMotion = warpTwoMotion(self.motionData[i], self.motionData[self.refIdx])
                registeredMotion.writeBVH()
        

