#####################################################################
## Quality Analysis of key frame extraction
#####################################################################
import numpy as np

class keyFrameinFileReader():
    def __init__(self, line):
        num = len(line)

        for index in range(num):
            if ".bvh" in line[index]:
                self.filename = line[index]
                
            if line[index] == "keyframes:":
                index = self.readKeyFrames(index, line)
                break
  
        index = self.readDoubleStance(index, line)
        index = self.readRightStance(index, line)
        index = self.readLeftStance(index, line)
        self.NumberofKeyFrames = len(self.keyFrames)
        
    def readKeyFrames(self,index, line):
        # index is the index of "keyframes:"
        self.keyFrames = []
        index = index + 1
        while line[index] != "doubleStance:":
             self.keyFrames.append(int(line[index]))
             index = index + 1
        return index
        
    def readDoubleStance(self, index, line):
        # index is the index of "doubleStance: "
        self.doubleStances = []
        index = index + 1
        while line[index] != "rightStance:":
            self.doubleStances.append(int(line[index]))
            index = index + 1
        return index

    def readRightStance(self, index, line):
        # index is the index of "rightStance:"
        self.rightStances = []
        index = index + 1
        while line[index] != "leftStance:":
            self.rightStances.append(int(line[index]))
            index = index + 1
        return index

    def readLeftStance(self, index, line):
        # index is the index of "leftStance:"
        self.leftStances = []
        index = index + 1
        while index < len(line):
            self.leftStances.append(int(line[index]))
            index = index + 1
        return index
    
manuallyLabeledMarkerlessFile = 'manually labeled key frame_markerless mocap data.txt'
automaticLabeledMarkerlessFile = 'automatically labeled key frame_markerless mocap data.txt'
manuallyLabeledWiCoFile = 'manually labeled key frame_WiCo mocap data.txt'
automaticLabeledWiCoFile = 'automatically labeled key frame_WiCo mocap data.txt'
###########################################################################
infile = open(manuallyLabeledMarkerlessFile, 'rb')
manuallyLabeledMarkerlessFileList = []
lines = infile.readlines()
totalNumberOfmanuallyLabeledMarkerless = 0
for line in lines:
    line = line.split()
    newFile = keyFrameinFileReader(line)
    totalNumberOfmanuallyLabeledMarkerless += newFile.NumberofKeyFrames
    manuallyLabeledMarkerlessFileList.append(newFile)
print "total number of manually Labeled key frames of markerless mocap data is :" + str(totalNumberOfmanuallyLabeledMarkerless)
############################################################################
infile = open(automaticLabeledMarkerlessFile, 'rb')
automaticLabeledMarkerlessFileList = []
lines = infile.readlines()
totalNumberOfautomaticLabeledMarkerless = 0
for line in lines:
    line = line.split()
    newFile = keyFrameinFileReader(line)
    totalNumberOfautomaticLabeledMarkerless += newFile.NumberofKeyFrames
    automaticLabeledMarkerlessFileList.append(newFile)
print "total number of automatic Labeled key frames of markerless mocap data is :" + str(totalNumberOfautomaticLabeledMarkerless)
############################################################################
infile = open(manuallyLabeledWiCoFile, 'rb')
manuallyLabeledWiCoFileList = []
lines = infile.readlines()
totalNumberOfmanuallyLabeledWiCo = 0
for line in lines:
    line = line.split()
    newFile = keyFrameinFileReader(line)
    totalNumberOfmanuallyLabeledWiCo += newFile.NumberofKeyFrames
    manuallyLabeledWiCoFileList.append(newFile)
print "total number of manually labeled key frames of Wicon mocap data is: " + str(totalNumberOfmanuallyLabeledWiCo)
############################################################################
infile = open(automaticLabeledWiCoFile, 'rb')
automaticLabeledWiCoFileList = []
lines = infile.readlines()
totalNumberOfautomaticLabeledWiCo = 0
for line in lines:
    line = line.split()
    newFile = keyFrameinFileReader(line)
    totalNumberOfautomaticLabeledWiCo += newFile.NumberofKeyFrames
    automaticLabeledWiCoFileList.append(newFile)
print "total number of automatically labeled key frames of Wicon mocap data is: " + str(totalNumberOfautomaticLabeledWiCo)
