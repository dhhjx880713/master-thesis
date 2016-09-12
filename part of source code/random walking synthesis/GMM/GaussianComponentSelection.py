# this script uses Cross Validation to select models
from MorphableModel import *

filepath = r'../motion decomposition/registered_pose2/*.bvh'
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
projectedFeatures = M.projectedFeatures.copy()
# randomly permute observations
projectedFeatures = np.random.permutation(projectedFeatures)
#projectedFeatures = np.random.permutation(projectedFeatures)
#K = [1,2,3,4,5,6,7,8,9,10]
##K = range(1,20)
##num = len(K)
##k_fold = 5
##numObs = len(projectedFeatures)
##len_fold = numObs/k_fold
##scores = np.zeros(num)
##for i in range(num):
##    # inital GMM model
##    g = mixture.GMM(n_components = K[i] , covariance_type = 'full')
##    
##    for j in range(k_fold):
##        if (j+1)*len_fold < numObs:
##            validation_set = projectedFeatures[j*len_fold:(j+1)*len_fold]
##        else:
##            validation_set = projectedFeatures[j*len_fold:]
##        if j == 0:
##            training_set = projectedFeatures[len_fold:]
##        else:
##            training_set = np.concatenate((projectedFeatures[:j*len_fold], projectedFeatures[(j+1)*len_fold:]))
##        g.fit(training_set)
##        #
##        scores[i] += sum(g.score(validation_set))
##    scores[i] = scores[i]/k_fold
##index = max(xrange(num), key = scores.__getitem__)
##print scores
##print index
##g = mixture.GMM(n_components = K[index], covariance_type = 'full')
##g.fit(projectedFeatures)
lowest_bic = np.infty
bic = []
num = 20
n_components_range = range(1,num)
for n_components in n_components_range:
    # Fit a mixture of gaussians with EM
    gmm = mixture.GMM(n_components=n_components, covariance_type='full')
    gmm.fit(projectedFeatures)
    bic.append(gmm.bic(projectedFeatures))
    if bic[-1] < lowest_bic:
        lowest_bic = bic[-1]
        best_gmm = gmm
bic = np.array(bic)
print bic
index = min(xrange(num-1), key = bic.__getitem__)
print index
