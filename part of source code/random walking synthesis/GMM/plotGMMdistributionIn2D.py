import sys
include_path = r'../tools/motion segment/'
sys.path.append(include_path)
from MorphableModel import *
import matplotlib.cm as cm
from RegisteredMotionSegment import *
def featureVectorToMotion(featureVector,frameNum):
    # the input featureVector should be an array list
    featureVector = featureVector.tolist()
    motionData = []
    index = 0
    num = len(featureVector)
    frameLen = num/frameNum
    while(index != num):
        if index + frameLen != num:
            motionData.append(featureVector[index:index+frameLen])
        else:
            motionData.append(featureVector[index:])
        index = index + frameLen
    return motionData
filepath = r'../motion decomposition/registered_pose3/*.bvh'
fileList = glob.glob(filepath)
M = MorphableModel(filepath)
#######################################################
# create weights
numFrames = M.mo_segments[0].numFrames
lenFeatures = len(M.mo_segments[0].data[0])
weight_vector = np.ones(lenFeatures)
# root joint
weight_vector[3:6] = [11,11,11]
# spine1
weight_vector[6:9] = [10,10,10]
# spine2
weight_vector[9:12] = [9,9,9]
# spine3
weight_vector[12:15] = [8,8,8]
# spine4
weight_vector[15:18] = [7,7,7]
# neck1
weight_vector[18:21] = [6,6,6]
# neck2
weight_vector[21:24] = [5,5,5]
# head_ee
weight_vector[24:27] = [4,4,4]
# left clavicle
weight_vector[27:30] = [6,6,6]
# left shoulder
weight_vector[30:33] = [5,5,5]
# left elbow
weight_vector[33:36] = [4,4,4]
# left lowarm twist
weight_vector[36:39] = [3,3,3]
# left hand
weight_vector[39:42] = [2,2,2]
# right clavicle
weight_vector[45:48] = [6,6,6]
# right shoulder
weight_vector[48:51] = [5,5,5]
# right elbow
weight_vector[51:54] = [4,4,4]
# right lowarm twist
weight_vector[54:57] = [3,3,3]
# right hand
weight_vector[57:60] = [2,2,2]
# left_hips
weight_vector[63:66] = [10,10,10]
# left_knee
weight_vector[66:69] = [9,9,9]
# left_ankle
weight_vector[69:72] = [8,8,8]
# left_toes
weight_vector[72:75] = [7,7,7]
# left_foot_ee
weight_vector[75:78] = [6,6,6]
# right_hips
weight_vector[78:81] = [10,10,10]
# right_knee
weight_vector[81:84] = [9,9,9]
# right_ankle
weight_vector[84:87] = [8,8,8]
# right_toes
weight_vector[87:90] = [7,7,7]
# right_foot_ee
weight_vector[90:] = [6,6,6]
#weight_vector = weight_vector
weight_vector = np.tile(weight_vector, numFrames)
# project motion data to subspace
M.createWeightedMorphableFunction(weight_vector)
##projectedFeatures = M.projectedFeatures
##plotData = projectedFeatures[:,:2]
M.sampleWeightedPCA(weight_vector)
newMotion = M.newMotion
MotionData = featureVectorToMotion(newMotion, numFrames)
m = MotionSegment(MotionData, fileList[0], type='data')
m.writeBVH()
m.display()
#m3.writeMP4()
########################################################
# plot plotData
##xmax = np.max(plotData[:,0])
##xmin = np.min(plotData[:,0])
##xlen = xmax - xmin
##ymax = np.max(plotData[:,1])
##ymin = np.min(plotData[:,1])
##ylen = ymax - ymin
##fig = plt.figure()
##plt.scatter(*plotData.T, marker = 'o', color = 'r')
##plt.axis([xmin - 0.1*xlen,xmax + 0.1*xlen,ymin - 0.1*ylen,ymax + 0.1*ylen])
##plt.show()
#########################################################
# train GMM based on plotData
# select number of Gaussian
##obs = plotData
##obs = np.random.permutation(obs)
##
##
##K = [1,2,3,4,5,6,7,8]
##num = len(K)
##k_fold = 5
##numObs = len(obs)
##len_fold = numObs/k_fold
##scores = np.zeros(num)
##for i in range(num):
##    # inital GMM model
##    g = mixture.GMM(n_components = K[i] , covariance_type = 'full')
##    
##    for j in range(k_fold):
##        validation_set = obs[j*len_fold:(j+1)*len_fold]
##        if j == 0:
##            training_set = obs[len_fold:]
##        else:
##            training_set = np.concatenate((obs[:j*len_fold], obs[(j+1)*len_fold:]))
##        g.fit(training_set)
##        #
##        scores[i] += sum(g.score(validation_set))
##    scores[i] = scores[i]/k_fold
##index = max(xrange(num), key = scores.__getitem__)
##print scores
##print index
################################################################
### train GMM on optimal parameter
##k = K[index]
##g = mixture.GMM(n_components = k)
##g.fit(obs)
##deltaX = ((xmax + 0.1*xlen) - (xmin - 0.1*xlen))/100
##deltaY = ((ymax + 0.1*ylen) - (ymin - 0.1*ylen))/100
##X = np.arange(xmin - 0.1*xlen,xmax + 0.1*xlen,deltaX)
##Y = np.arange(ymin - 0.1*ylen,ymax + 0.1*ylen,100)
##X,Y = np.meshgrid(X,Y)
####point = [X[0,0], Y[0,0]]
####print g.score([point,])
##xdim, ydim = X.shape
##Z = np.zeros([xdim,ydim])
##for i in range(xdim):
##    for j in range(ydim):
##        point = [X[i,j], Y[i,j]]
##        Z[i,j] = g.score([point,])
##plt.figure()
##im = plt.imshow(Z, interpolation = 'bilinear', origin = 'lower', cmap = cm.gray,\
##                extent = [xmin - 0.1*xlen,xmax + 0.1*xlen,ymin - 0.1*ylen,ymax + 0.1*ylen])
###plt.axis([xmin - 0.1*xlen,xmax + 0.1*xlen,ymin - 0.1*ylen,ymax + 0.1*ylen])
##plt.scatter(*plotData.T, marker = 'o', color = 'r')
##plt.show()
