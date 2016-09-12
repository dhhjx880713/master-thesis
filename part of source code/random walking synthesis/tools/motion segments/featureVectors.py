import os, sys
from skeleton import *
import numpy as np
sys.path.append(r'../time warpping/')
from warpping import *
from PCA import *
from visualizer import *
from MotionPrimitives import *

def extractDataFromJoints(joint,index,oneFrameData = {}):
    oneFrameData[joint.name] = [joint.worldpos[index][0],joint.worldpos[index][2],joint.worldpos[index][1]]
    if len(joint.children) != 0:
        for child in joint.children:
            oneFrameData = extractDataFromJoints(child,index, oneFrameData)
    return oneFrameData

def plotOneFrame(frame):
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    ax.plot(*frame)
    ax.set_xlim3d(-1000, 1000)
    ax.set_ylim3d(-1000, 1000)
    ax.set_zlim3d(-1000, 1000)
   
    plt.show()
    
def Error(vec1, vec2):
    if len(vec1) != len(vec2):
        raise ValueError('two vectors should have same length')
    n = len(vec1)
    err = 0
    for i in range(n):
        err += (vec1[i] - vec2[i])**2
    err = err/n
    return err

warppedVecIndex = DTW(samples)
# export data of motion segments
outfile = file('segments.txt','w')
for index in warppedVecIndex:
    outfile.write('motion segment: \n')
    for item in warppedVecIndex[index]:
        for value in mybvh.keyframes[item]:
            outfile.write("%s " % str(value))
        outfile.write('\n')
    outfile.write('\n')
outfile.close()
num = len(warppedVecIndex)
features = []  # feature vectors
##start_frame_index = warppedVecIndex[0][0]
##tmp = mybvh.keyframes[start_frame_index]
##print mybvh.keyframes[start_frame_index]
##refXYZ = [tmp[0],tmp[1],tmp[2]]
##for i in warppedVecIndex[0]:
##    print refXYZ
##    Vec = mybvh.keyframes[i]
##    # normalize the x,y,z position of root for each frame
##    Vec[0] = Vec[0] - refXYZ[0]
##    Vec[1] = Vec[1] - refXYZ[1]
##    Vec[2] = Vec[2] - refXYZ[2]
##    tmp = tmp + Vec
##print mybvh.keyframes[start_frame_index]
for item in warppedVecIndex:
    start_frame_index = warppedVecIndex[item][0]
    tmp = mybvh.keyframes[start_frame_index]
    refXYZ = [tmp[0],tmp[1],tmp[2]]  
    tmp = []
    for i in warppedVecIndex[item]:
        
        Vec = mybvh.keyframes[i][:]
        # normalize the x,y,z position of root for each frame
        Vec[0] = Vec[0] - refXYZ[0]
        Vec[1] = Vec[1] - refXYZ[1]
        Vec[2] = Vec[2] - refXYZ[2]
        tmp = tmp + Vec
    features.append(tmp)


########################################
# display one motion sample features[0]
# input: features[0]
# feature[0] contains all data for motion primitive one
# firstly, parsing feature[i], reconstructing frame data
index = 0
frameLen = len(mybvh.keyframes[0])
frameNum = len(warppedVecIndex[0])
newFrames = {}
tmp = features[index][:]
#########################################
## generate the skeleton data for display
skeletonData = {}
for i in range(len(warppedVecIndex[index])):
    newFrames[i] = tmp[0:frameLen]
    process_bvhkeyframe(newFrames[i],hips,i,0)
    oneFrameData = extractDataFromJoints(hips,i)
    #extract data according to joints sequency
    skeletonData[i] = []
    for joint in skeleton_body_mpi:
        if joint in oneFrameData:
            skeletonData[i].append(oneFrameData[joint])
    skeletonData[i] = np.array(skeletonData[i])
    skeletonData[i] = np.transpose(skeletonData[i])
    tmp = tmp[frameLen:]
   
##scene_range = [0,len(warppedVecIndex[index])-1]
##Visualize(skeletonData, scene_range)
##fig = plt.figure()
##ax = p3.Axes3D(fig)
##for i in np.arange(*scene_range):
##    ax.plot(*skeletonData[i])
##ax.set_xlim3d(-1000, 1000)
##ax.set_ylim3d(-1000, 1000)
##ax.set_zlim3d(-1000, 1000)
##plt.show()
#############################################

##############################################
## generate feature matrix for PCA
feature_mat = np.matrix(np.array(features))
C = Center(feature_mat,verbose=0)
fraction = 0.95
moPri = features[0]
M = MotionPrimitive(features[0],frameLen, frameNum)
##p = PCA(np.transpose(feature_mat),fraction = fraction)
###############################################
#### build the projection matrix
####n = p.npc
####projection_mat = p.U[:,:n]
##projection_mat = p.U
################################################
### project features[0] to the new space
##projected_feature = np.dot(projection_mat, features[0])
##
##new_feature = np.dot(np.transpose(projection_mat), projected_feature)
##new_feature, = np.array(new_feature.T)
##print new_feature
###M = MotionPrimitive(new_feature, frameLen,frameNum )
##U,D,V = np.linalg.svd(feature_mat,full_matrices=True)
##S = np.zeros((26,4002), dtype = complex)
##S[:26, :26] = np.diag(S)
##testData = feature_mat[0,:]
##testData = np.transpose(testData)
##testData = np.array(testData)
##err = []
##for i in range(4002):
##    projection = V[:,:4002-i]
##    projectedVec = np.dot(np.transpose(projection), testData)
##    backpro = np.dot(projection, projectedVec)
##    backpro = np.array(backpro)
##    err.append(Error(testData, backpro))
##fig = plt.figure()
##plt.plot(err)
##plt.show()
