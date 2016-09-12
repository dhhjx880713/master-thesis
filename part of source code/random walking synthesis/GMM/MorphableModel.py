import sys
include_path = r'../tools/motion segments/'
sys.path.append(include_path)
from MotionSegment import *
import glob
from sklearn import mixture
from PCA import *

weight_vector = np.ones(93)
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


class MorphableModel():
    
    def __init__(self, *args, **kwargs):
        if 'type' in kwargs:
            type = kwargs.pop('type')
        else:
            raise ValueError('''Input data type must be specified. Either type = 'MotionSegments' or type = 'files' ''')
        if type == 'MotionSegments':
            self.mo_segments = args[0]
            self.getFeatures()
        elif type == 'files':
            self.fileList = glob.glob(args[0])
            if self.fileList == []:
                raise ValueError('loading file failed!')
            self.getMotionSegments()
            self.getFeatures()
        else:
            raise ValueError('Unknown input data type!')
        self.NumOfMotionSegments = len(self.mo_segments)
        self.getWeights(weight_vector)

    def getMotionSegments(self):
        self.mo_segments = []
        for item in self.fileList:
            self.mo_segments.append(MotionSegment(item, type = 'file'))

    def getFeatures(self):
        self.mo_features = []
        for motion in self.mo_segments:
            feature_vec = []
            for frame in motion.data:
                feature_vec += frame
            self.mo_features.append(feature_vec)
            
    def getWeights(self, weights):
        numFrames = self.mo_segments[0].numFrames
        extendedWeightVector = []
        for i in range(numFrames):
            extendedWeightVector = np.concatenate((extendedWeightVector, weight_vector))
        self.weights = extendedWeightVector

    def createMorphableFunction(self):
        self.num = len(self.mo_features)
        features = np.array(self.mo_features)
        
        #print features.shape
        #features = features/np.sqrt(self.num)
        # centralize the data
        c = Center(features)
        
        self.mean = np.ravel(c.mean)
        p = PCA(features, fraction = 0.999)

        self.eigVectors = p.Vt[:p.npc]
        #self.eigVectors = p.Vt[:]
        # project features into subpace
        self.projectedFeatures = []
        for item in features:
            projectedFeature = np.dot(self.eigVectors, item)
            self.projectedFeatures.append(projectedFeature)
        self.projectedFeatures = np.array(self.projectedFeatures)
        
    def backprojectionData(self):
        self.backprojections = []

        for item in self.projectedFeatures:
            backprojection = np.dot(np.transpose(self.eigVectors), item)
            backprojection = np.ravel(backprojection)
            # add mean of original data
            backprojection += self.mean
            self.backprojections.append(backprojection)
            
            
        self.backprojections = np.array(self.backprojections)

    def backprojectionWeightedData(self):
        self.backprojections = []
        for item in self.projectedFeatures:
            backprojection = np.dot(np.transpose(self.eigVectors), item)
            backprojection = np.ravel(backprojection)
            # rescale the data back
            backprojection = backprojection * (1/self.weights)
            backprojection += self.mean
            self.backprojections.append(backprojection)
        self.backprojections = np.array(self.backprojections)
        
        
    def createWeightedMorphableFunction(self):
        self.num = len(self.mo_features)
        features = np.array(self.mo_features)
        #print features.shape
        #features = features/np.sqrt(self.num)
        # centralize the data
        c = Center(features)
        
        self.mean = np.ravel(c.mean)
        ## rescale features
        rescaled_features = []
        for item in features:
            new_vec = item *self.weights
##            for index in len(item):
##                newValue = item[index]*weights[index]
##                new_vec.append(newValue)

            rescaled_features.append(new_vec)
        rescaled_features = np.array(rescaled_features)
        
        p = PCA(rescaled_features, fraction = 0.99)
        self.eigVectors = p.Vt[:p.npc]
        #self.eigVectors = p.Vt
        # project rescaled data
        self.projectedFeatures = []
        for item in rescaled_features:
            projectedFeature = np.dot(self.eigVectors, item)
            self.projectedFeatures.append(projectedFeature)
        self.projectedFeatures = np.array(self.projectedFeatures)
        

    def projection(self, feature_vector):
        # feature_vector is not centralized
        feature_vector = np.array(feature_vector)
        #print feature_vector
        self.centralizedFeatureVector = feature_vector - self.mean
#         if weights!= []:
#             # rescale centralizeFeatureVector
#             self.centralizedFeatureVector = self.centralizedFeatureVector * weights
        self.projectedFeatureVector = np.dot(self.eigVectors, self.centralizedFeatureVector.T)
        self.projectedFeatureVector = np.ravel(self.projectedFeatureVector)

    def backprojection(self, projectedFeatureVector):
        projectedFeatureVector = np.array(projectedFeatureVector)
        self.backprojectedFeatureVector = np.dot(np.transpose(self.eigVectors),projectedFeatureVector.T)
        self.backprojectedFeatureVector = np.ravel(self.backprojectedFeatureVector)
        self.backprojectedFeatureVector += self.mean
        #self.backprojectedFeatureVector = self.backprojectedFeatureVector*np.sqrt(self.num)
        self.backprojectedFeatureVector.tolist()
    
    def weightedSample(self, projectedFeatureVector):
        projectedFeatureVector = np.array(projectedFeatureVector)
        self.backprojectedFeatureVector = np.dot(np.transpose(self.eigVectors),projectedFeatureVector.T)
        self.backprojectedFeatureVector = np.ravel(self.backprojectedFeatureVector)
        self.backprojectedFeatureVector = self.backprojectedFeatureVector * (1/self.weights)
        self.backprojectedFeatureVector += self.mean   
        self.backprojectedFeatureVector.tolist()

    def trainGaussianMixtureModel(self):
        lowest_bic = np.infty
        bic = []
        num = 40
        n_components_range = range(1,num)
        for n_components in n_components_range:
            gmm = mixture.GMM(n_components=n_components, covariance_type='full')
            gmm.fit(self.projectedFeatures)
            bic.append(gmm.bic(self.projectedFeatures))
            if bic[-1] < lowest_bic:
                lowest_bic = bic[-1]
                self.g = gmm
                
    def sample(self):
        
        #g = mixture.GMM(n_components = 8 , covariance_type = 'full')
        #g.fit(self.projectedFeatures)
        
    
        self.newFeature = self.g.sample()
        self.newFeature = np.ravel(self.newFeature)
##        self.backprojection(self.newFeature)
##        self.newMotion =  self.backprojectedFeatureVector
        self.weightedSample(self.newFeature)
        self.newMotion = self.backprojectedFeatureVector
    
    def sampleWeightedPCA(self):
        #self.g = mixture.GMM(n_components = 7 , covariance_type = 'full')
        #self.g.fit(self.projectedFeatures)
        #self.newFeature = g.sample()
        print "sample statistical model"
        self.GMMsample()
        

    def GMMsample(self):
        index = max(xrange(len(self.g.weights_)), key = self.g.weights_.__getitem__)
        mean = self.g.means_[index]
        lenOfMotion = len(mean)
        covar = self.g.covars_[index]
        U,s,V = np.linalg.svd(covar)
        sqrtS = np.diag(np.sqrt(s))
        sqrt_covar = np.dot(U, np.dot(sqrtS, V))
        rand = np.random.randn(lenOfMotion)
        rand = np.dot(sqrt_covar, rand) * 0.5
        #rand = rand 
        self.newFeature = rand.T + mean
        print "new motion!"
        #self.newFeature = mean
        self.newFeature = np.ravel(self.newFeature)
        self.weightedSample(self.newFeature)
        self.newMotion = self.backprojectedFeatureVector
        
    def randomSample(self):
        
        index = np.random.randint(self.NumOfMotionSegments)
        newMotion = self.mo_segments[index]
        return newMotion
    
def main():
    filepath = r'../mocap_data/registered_pose2/*.bvh'
    M = MorphableModel(filepath, type = 'files')
    M.createWeightedMorphableFunction()
    M.trainGaussianMixtureModel()
    M.sampleWeightedPCA()
    #M.sample()
    newMotion = M.newMotion
    frameNum = M.mo_segments[0].numFrames
    MotionData = featureVectorToMotion(newMotion, frameNum)
    typeFile = M.fileList[0]
    m3 = MotionSegment(MotionData, typeFile, type='data')
    m3.display()

if __name__ == "__main__":
    main()
