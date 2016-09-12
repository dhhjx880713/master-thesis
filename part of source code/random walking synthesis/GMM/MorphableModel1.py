import sys
include_path = r'../tools/motion segments/'
sys.path.append(include_path)
from MotionSegment import *
import glob
from sklearn import mixture
from PCA import *

class MorphableModel1():
##    def __init__(self, directoryWithSegmentedBVHFiles):
##        self.fileList = glob.glob(directoryWithSegmentedBVHFiles)
##        #self.fileList = fileList
##        self.getMotionSegments()
##        self.getFeatures()
##        self.createMorphableFunction()
##
##    def getMotionSegments(self):
##        self.mo_segments = []
##        for item in self.fileList:
##            self.mo_segments.append(MotionSegment(item, type = 'file'))
    def __init__(self, motionSegments):
        self.mo_segments = motionSegments
        self.getFeatures()
        self.createMorphableFunction()

    def getFeatures(self):
        self.mo_features = []
        for motion in self.mo_segments:
            feature_vec = []
            for frame in motion.data:
                feature_vec += frame
            self.mo_features.append(feature_vec)

    def createMorphableFunction(self):
        features = np.mat(self.mo_features)
        #print features.shape
        c = Center(features)
        self.mean = np.ravel(c.mean)
        p = PCA(features, fraction = 0.999)
        self.eigVectors = p.Vt[:p.npc]
        
        

    def projection(self, feature_vector):
        # feature_vector is not centralized
        feature_vector = np.array(feature_vector)
        #print feature_vector
        centralizedFeatureVector = feature_vector - self.mean
        self.projectedFeatureVector = np.dot(self.eigVectors, centralizedFeatureVector.T)
        self.projectedFeatureVector = np.ravel(self.projectedFeatureVector)

    def backprojection(self, projectedFeatureVector):
        projectedFeatureVector = np.array(projectedFeatureVector)
        self.backprojectedFeatureVector = np.dot(np.transpose(self.eigVectors),projectedFeatureVector.T)
        self.backprojectedFeatureVector = np.ravel(self.backprojectedFeatureVector)
        self.backprojectedFeatureVector += self.mean
        self.backprojectedFeatureVector.tolist()
        
    def sample(self):
        
        g = mixture.GMM(n_components = 1 , covariance_type = 'full')
        g.fit(np.array(self.mo_features))
        
      
        self.newFeature = g.sample()
        self.newFeature = np.ravel(self.newFeature)
