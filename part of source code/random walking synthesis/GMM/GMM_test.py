import numpy as np
from sklearn import mixture
# set the random seed
np.random.seed(1)
# generate observations, two Gaussian Distribution
data1 = np.random.randn(200,30)
data2 = np.random.randn(300,30) + 10
obs = np.concatenate((data1, data2))
obs = np.random.permutation(obs)
print type(obs)
# initalize GMM model
##K = [1,2,3,4,5,6,7]
##num = len(K)
##scores = np.zeros(num)
##for i in range(num):
##    g = mixture.GMM(n_components = K[i], covariance_type = 'full')
##    # Use EM Algorithm to train GMM 
##    g.fit(obs)
##    scores[i] = sum(g.score(obs))
##print scores
##index = max(xrange(num), key = scores.__getitem__)
##print index
##g = mixture.GMM(n_components = K[index], covariance_type = 'full')
##g.fit(obs)
##predictions = g.predict(obs)
##print predictions
##g = mixture.GMM(n_components = 2, covariance_type = 'full')
##g.fit(obs)
##predictions = g.predict(obs)
##print predictions

# initalize GMM model
K = [1,2,3,4,5,6,7,8]
num = len(K)
k_fold = 5
numObs = len(obs)
len_fold = numObs/k_fold
scores = np.zeros(num)
for i in range(num):
    # inital GMM model
    g = mixture.GMM(n_components = K[i] , covariance_type = 'full')
    
    for j in range(k_fold):
        validation_set = obs[j*len_fold:(j+1)*len_fold]
        if j == 0:
            training_set = obs[len_fold:]
        else:
            training_set = np.concatenate((obs[:j*len_fold], obs[(j+1)*len_fold:]))
        g.fit(training_set)
        #
        scores[i] += sum(g.score(validation_set))
    scores[i] = scores[i]/k_fold
index = max(xrange(num), key = scores.__getitem__)
print scores
print index
##print g.weights_
##print g.means_
##print g.covars_
##t = np.random.randn(10,3)
##print t
