""" class for weighted Principal Component Analysis
Usage:
    p = PCA( A, w, fraction=0.90 )
In:
    A: an array of e.g. 1000 observations x 20 variables, 1000 rows x 20 columns
    w: an array of weights for features
    fraction: use principal components that account for e.g.
        90 % of the total variance

Out:
    p.U, p.d, p.Vt: from numpy.linalg.svd, A = U . d . Vt
    p.dinv: 1/d or 0, see NR
    p.eigen: the eigenvalues of A*A, in decreasing order (p.d**2).
        eigen[j] / eigen.sum() is variable j's fraction of the total variance;
        look at the first few eigen[] to see how many PCs get to 90 %, 95 % ...
    p.npc: number of principal components,
        e.g. 2 if the top 2 eigenvalues are >= `fraction` of the total.
        It's ok to change this; methods use the current value.
from __future__ import division
import numpy as np
dot = np.dot
"""


class weighted_PCA:
    def __init__( self, A, w, fraction=0.90 ):
        assert 0 <= fraction <= 1
            # A = U . diag(d) . Vt, O( m n^2 ), lapack_lite --
        new_vecs = []
        for item in A:
            temp = np.ravel(item)
            new_vec = temp*w
            new_vecs.append(new_vec)
        new_vecs = np.mat(new_vecs)
        
        self.U, self.d, self.Vt = np.linalg.svd( new_vecs, full_matrices=True )
        assert np.all( self.d[:-1] >= self.d[1:] )  # sorted
        self.eigen = self.d**2
        self.sumvariance = np.cumsum(self.eigen)
        self.sumvariance /= self.sumvariance[-1]
        self.npc = np.searchsorted( self.sumvariance, fraction ) + 1
        self.dinv = np.array([ 1/d if d > self.d[0] * 1e-6  else 0
                                for d in self.d ])
        eigVecs = []
        for item in self.Vt:
            temp = np.ravel(item)
            new_vec = temp*(1/w)
            eigVecs.append(new_vec)
        self.Vt = np.mat(eigVecs)

    def pc( self ):
        """ e.g. 1000 x 2 U[:, :npc] * d[:npc], to plot etc. """
        n = self.npc
        #return self.U[:, :n] * self.d[:n]
        return np.dot(self.U[:,:n], self.d[:n])

    # These 1-line methods may not be worth the bother;
    # then use U d Vt directly --

    def vars_pc( self, x ):
        n = self.npc
        return self.d[:n] * dot( self.Vt[:n], x.T ).T  # 20 vars -> 2 principal

    def pc_vars( self, p ):
        n = self.npc
        return dot( self.Vt[:n].T, (self.dinv[:n] * p).T ) .T  # 2 PC -> 20 vars

    def pc_obs( self, p ):
        n = self.npc
        return dot( self.U[:, :n], p.T )  # 2 principal -> 1000 obs

    def obs_pc( self, obs ):
        n = self.npc
        return dot( self.U[:, :n].T, obs ) .T  # 1000 obs -> 2 principal

    def obs( self, x ):
        return self.pc_obs( self.vars_pc(x) )  # 20 vars -> 2 principal -> 1000 obs

    def vars( self, obs ):
        return self.pc_vars( self.obs_pc(obs) )  # 1000 obs -> 2 principal -> 20
