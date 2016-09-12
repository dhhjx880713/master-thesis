import sys
include_path = r'../tools/motion segments/'
sys.path.append(include_path)
from MotionSegment import *
from RegisteredMotionSegment import *
from distFrames import *
from transformations import *

def getSkeleton(joint, skeleton = []):
    # recursively search joints to get the whole description about skeleton
    for child in joint.children:
        skeleton.append([joint.name, child.name])
    for child in joint.children:
        skeleton = getSkeleton(child, skeleton)
    return skeleton

def displayFrames(hips, framesWorldpos):
    # visualize selected frames based on input skeleton
    # get skeleton from hips
    skeleton = getSkeleton(hips)
    fig = plt.figure()
    ax =p3.Axes3D(fig)
    origin =framesWorldpos[0]['root']
    for frameIndex in framesWorldpos:
        for item in skeleton:
            tmp = [framesWorldpos[frameIndex][item[0]], framesWorldpos[frameIndex][item[1]]]
            tmp = np.transpose(np.array(tmp))
            ax.plot(*tmp, color = 'b')
    ax.set_xlim3d(-2000+origin[0], 2000+origin[0])
    ax.set_ylim3d(-2000+origin[1], 2000+origin[1])
    ax.set_zlim3d(-1000+origin[2], 2000+origin[2])
    #self.ax.text2D(0.05, 0.95, 'frame: '+ str(frame_num), transform = ax.transAxes)
    plt.show()

def distance(worldpos1, worldpos2):
    dist = 0
    for item in worldpos1:
        vec1 = np.array(worldpos1[item])
        vec2 = np.array(worldpos2[item])
        dist += np.linalg.norm(vec1-vec2)
    return dist

def applyRotation(worldpos, theta):
    # the root point of worldpos must be (0,0,0)
    # theta is radian 
    if worldpos['root'] != [0,0,0]:
        raise ValueError('root point must at (0,0,0)')
    # construct rotation matrix
    rotmat = np.identity(4)
    rotmat[0,0] = np.cos(theta)
    rotmat[0,1] = np.sin(theta)
    rotmat[1,0] = -np.sin(theta)
    rotmat[1,1] = np.cos(theta)
    newWorldpos = {}
    for item in worldpos:
        point = np.ones(4)
        point[:-1] = worldpos[item][:]
        point = np.mat(point).T
        newpoint = rotmat*point
        tmp = newpoint[:-1]
        tmp = np.array(tmp).T
        newWorldpos[item] = tmp[0].tolist()
    return newWorldpos

##def pathSearch(costMat, DEBUG = 0):
##    # finding the shortest path from the cost matrix
##    # the row indexes are frame index of reference motion
##    # the column indexes are frame index of test motion which is to be warpped
##    # three rules for warpping:
##    # 1. Continuity: each cell on the path must share a corner or edge with another cell on the path
##    # 2. Causality: paths must not reverse direction
##    # 3. Slope limit: At most 2 consecutive horizontal steps or vertical steps could be taken
##    m,n = costMat.shape
##    path = np.zeros([m-1, n-1])
##    path[0,0] = 1
##    #counterVertical = 0 # counter for counting how many steps in horizontal or vertical direction
##    counterHorizontal = 0
##    i = 1
##    j = 1
##    #counter = 0
##    while j!= n-1:
##        if i!= m-1:
##            
##
##            if counterHorizontal == 1:
##                # go to diagonal
##                # searchArea = [costMat[i,j+1], costMat[i+1,j+1]]
##                
##                i = i + 1
##                j = j + 1
##                counterHorizontal = 0
##            
##            else:
##               if costMat[i, j+1] >= costMat[i+1, j+1]:
##               
##                   # go to diagonal
##                   i = i + 1
##                   j = j + 1
##               else:
##                
##                   # go to horizontal
##                   i = i 
##                   j = j + 1
##                   counterHorizontal += 1
##            path[i-1, j-1] = 1
##        else :
##            j = j + 1
##            path[i-1, j-1] = 1
##            #counterHorizontal += 1
##    if DEBUG:        
##        plt.imshow(path)
##        plt.show()
##    return path
def searchPath(simMat, DEBUG = 0):
    # finding the shortest path from the similarity matrix
    # the row indexes are frame index of reference motion
    # the column indexes are frame index of test motion which is to be warpped
    m,n = simMat.shape
    path = np.zeros([m,n])
    path[0,0] = 1
    i = 0
    j = 0
    while i != m-1 and j!= n-1: # not at the boundary
        searchArea = [simMat[i+1,j], simMat[i+1, j+1], simMat[i, j+1]]
        minIndex = min(xrange(len(searchArea)), key = searchArea.__getitem__)
        if minIndex == 0:
            i = i + 1
            j = j
        elif minIndex == 1:
            i = i + 1
            j = j + 1
        else:
            i = i
            j = j + 1
        path[i,j] = 1
    if i == m-1 and j != n-1:
        while j < n-1:
            j = j + 1
            path[i,j] = 1
    if i != m-1 and j == n-1:
        while i < m-1:
            i = i + 1
            path[i,j] = 1
    if DEBUG:
        plt.imshow(path)
        plt.xlabel('reference motion')
        plt.ylabel('test motion')
        plt.show()
  
    return path
        
def pathSearch(costMat,DEBUG = 0):
    # finding the shortest path from the cost matrix
    # the row indexes are frame index of reference motion
    # the column indexes are frame index of test motion which is to be warpped
    # three rules for warpping:
    # 1. Continuity: each cell on the path must share a corner or edge with another cell on the path
    # 2. Causality: paths must not reverse direction
    # 3. Slope limit: At most 2 consecutive horizontal steps or vertical steps could be taken
    m,n = costMat.shape

    slopeLimit = 2.0
    if m > n:
        horizontalLimit = slopeLimit
        verticalLimit = np.ceil(slopeLimit*m/n)
    else:
        verticalLimit = slopeLimit
        horizontalLimit = np.ceil(slopeLimit*n/m)

    path = np.zeros([m-1, n-1])
    path[0,0] = 1
    counterVertical = 0 # counter for counting how many steps in horizontal or vertical direction
    counterHorizontal = 0
    i = 1
    j = 1
    
    while i!= m-1 and j!=n-1: # not at the boundary
        if counterVertical == verticalLimit:
            # go to horizontal or diagonal 
            # searchArea = [costMat[i+1,j], costMat[i+1,j+1]]
            if costMat[i+1,j+1] >= costMat[i,j+1]:
                # go one step at horizontal direction
                i = i
                j = j + 1
                counterHorizontal += 1
            else:
                i = i + 1
                j = j + 1
                
            counterVertical = 0

        elif counterHorizontal == horizontalLimit:
            # go to vertical or diagonal
            # searchArea = [costMat[i,j+1], costMat[i+1,j+1]]
            if costMat[i+1,j+1] >= costMat[i+1, j]:
                i = i + 1 
                j = j 
                counterVertical += 1
            else:
                i = i + 1
                j = j + 1
                
            counterHorizontal = 0
        else:
            searchArea = [costMat[i+1,j], costMat[i+1,j+1], costMat[i,j+1]]
            minIndex = min(xrange(len(searchArea)), key = searchArea.__getitem__)
            if minIndex == 0:
                i = i + 1
                j = j
                counterVertical += 1
            elif minIndex == 1:
                i = i + 1
                j = j + 1
            else:
                i = i
                j = j + 1
                counterHorizontal += 1
        path[i-1, j-1] = 1
    if i== m-1 and j!= n-1:
        while j< n-1:
            j = j + 1
            path[i-1,j-1] = 1
            
    if i!= m-1 and j== n-1:
        while i< m-1:
            i = i + 1
            path[i-1, j-1] = 1
    if DEBUG:
        plt.imshow(path)
        plt.xlabel('reference motion')
        plt.ylabel('test motion')
        plt.show()
  
    return path
                
            
          
    
# distance grid between two motion
##def warpTwoMotion(mo1, mo2, DEBUG = 0):
##    # compute the distance between two motions, 
##    
##    ## initial a matrix to store distance
##    distGrid = np.zeros([mo1.numFrames, mo2.numFrames])
##    thetaGrid = np.zeros([mo1.numFrames, mo2.numFrames])
##    for i in mo1.worldpos:
##        for j in mo2.worldpos:
##            dist, theta = distFrames(mo1.worldpos[i], mo2.worldpos[j])
##            distGrid[i,j] = dist
##            thetaGrid[i,j] = theta
##   
##    if DEBUG:
##        plt.imshow(distGrid)
##        plt.show()
##    costMat = costMatrix(distGrid, DEBUG)
##    #print costMat
##    path = pathSearch(costMat, DEBUG)
##    # find index of warpped motion
##    warppedVecIndex = []
##    for i in range(mo2.numFrames):
##        warppedVecIndex.append(max(xrange(len(path[:,i])), key = path[:,i].__getitem__))
##    if DEBUG:
##        print warppedVecIndex
##    return warppedVecIndex

##def warpTwoMotion(mo1, mo2, DEBUG = 0):
##    # warp test motion to reference motion
##    # input test motion mo1, reference motion mo2
##    # output registered motion
##    # initial a matrix to store distance
####    distGrid = np.zeros([mo1.numFrames, mo2.numFrames])
####    thetaGrid = np.zeros([mo1.numFrames, mo2.numFrames])
##    # if mo1 is longer than mo2, downsampling mo1
##    worldpos = {}
##    if mo1.numFrames > mo2.numFrames:
##        factor = np.floor(mo1.numFrames/mo2.numFrames)
##        if factor>1:
##            counter = 0
##            
##            sampleIndex = []
##            for i in mo1.worldpos:
##                if i%factor == 0:
##                    worldpos[counter] = mo1.worldpos[i]
##                    counter = counter + 1
##                    sampleIndex.append(i)
##            #print sampleIndex
##        else:
##            sampleIndex = np.arange(mo1.numFrames).tolist()
##            worldpos = mo1.worldpos
##    elif mo1.numFrames < mo2.numFrames:
##        factor = np.floor(mo2.numFrames/mo1.numFrames)
##        if factor > 1:
##            counter = 0
##            
##            sampleIndex = []
##            for i in mo1.worldpos:
##                for j in range(factor):
##                    worldpos[counter] = mo1.worldpos[i]
##                    counter = counter + 1
##                    sampleIndex.append(i)
##            #print sampleIndex
##        else:
##            sampleIndex = np.arange(mo1.numFrames).tolist()
##            worldpos = mo1.worldpos
##    else:
##        sampleIndex = np.arange(mo1.num).tolist()
##        worldpos = mo1.worldpos
##    print sampleIndex
####    for i in mo1.worldpos:
####        for j in mo2.worldpos:
####            dist, theta = distFrames(mo1.worldpos[i], mo2.worldpos[j])
####            distGrid[i,j] = dist
####            thetaGrid[i,j] = theta  # theta is radians
##    testMotionLen = len(worldpos)
##    distGrid = np.zeros([testMotionLen, mo2.numFrames])
##    thetaGrid = np.zeros([testMotionLen, mo2.numFrames])
##    for i in worldpos:
##        for j in mo2.worldpos:
##            dist, theta = distFrames(worldpos[i], mo2.worldpos[j])
##            distGrid[i,j] = dist
##            thetaGrid[i,j] = theta  # theta is radians
##    if DEBUG:
##        plt.imshow(distGrid)
##        plt.xlabel('reference motion')
##        plt.ylabel('test motion')
##        plt.show()
##        # export similarity matrix
####        outfile = open('similarityMatrix.txt', 'w')
####        m,n = distGrid.shape
####        for i in range(m):
####            for value in distGrid[i,:]:
####                outfile.write('%s ' % str(value))
####            outfile.write('\n')
####        outfile.close()
##        #print thetaGrid
##    
##    # create a new motion segment for registered motion segment
##    registered_mo = RegisteredMotionSegment(mo1)
##    registered_mo.getRotationAngle(thetaGrid[0,0])
##    registered_mo.getOrigin(mo2.data[0][:3])
##    # create cost matrix
##    costMat = costMatrix(distGrid, DEBUG)
##    path = pathSearch(costMat, DEBUG)
##    # find index of warpped motion
####    warppedVecIndex = []
####    for i in range(mo2.numFrames):
####        warppedVecIndex.append(max(xrange(len(path[:,i])), key = path[:,i].__getitem__))
##    temp = getWarpIndex(path)
##    print temp
##    warppedVecIndex = []
##    for i in range(len(temp)):
##        warppedVecIndex.append(sampleIndex[temp[i]])
##    print warppedVecIndex
##    
##    alignmentCurve = getAlignmentCurve(thetaGrid, temp)
##    registered_mo.getWarppingIndex(warppedVecIndex)
##    registered_mo.getAlignmentCurve(alignmentCurve)
##    registered_mo.smoothAlignmentCurve()
##    # transform and rotate test motion
##    registered_mo.transformation()
##    return registered_mo

def warpTwoMotion(mo1, mo2, weighted = 0, DEBUG = 0):
    # warp test motion to reference motion
    # input test motion mo1, reference motion mo2
    # output registered motion
    # initial a matrix to store distance
    distGrid = np.zeros([mo1.numFrames, mo2.numFrames])
    thetaGrid = np.zeros([mo1.numFrames, mo2.numFrames])
##    for i in mo1.worldpos:
##        for j in mo2.worldpos:
##            dist, theta = distFrames(mo1.worldpos[i], mo2.worldpos[j])
##            distGrid[i,j] = dist
##            thetaGrid[i,j] = theta
    window_size = 5
    k = int(np.floor(window_size/2))
    extendedMotion1 = {}
    extendedMotion2 = {}
    for index in range(mo1.numFrames):
        extendedMotion1[index+k] = mo1.worldpos[index].copy()
    for index in range(k):
        extendedMotion1[index] = mo1.worldpos[0].copy()
    
        extendedMotion1[mo1.numFrames+2*k-1-index] = mo1.worldpos[mo1.numFrames-1].copy()
    for index in range(mo2.numFrames):
        extendedMotion2[index+k] = mo2.worldpos[index].copy()
    for index in range(k):
        extendedMotion2[index] = mo2.worldpos[0].copy()
        extendedMotion2[mo2.numFrames+2*k-1-index] = mo2.worldpos[mo2.numFrames-1].copy()

    for i in mo1.worldpos:
        for j in mo2.worldpos:
            # construct motion neighbour
            motion1 = {}
            motion2 = {}
            for n in range(window_size):
                motion1[n] = extendedMotion1[i+ n].copy()
                motion2[n] = extendedMotion2[j+ n].copy()
            dist, theta = distFrames(motion1, motion2, weighted)
            distGrid[i,j] = dist
            #print dist
            thetaGrid[i,j] = theta


    if DEBUG:
        plt.imshow(distGrid)
        plt.show()
        # export similarity matrix
##        outfile = open('similarityMatrix.txt', 'w')
##        m,n = distGrid.shape
##        for i in range(m):
##            for value in distGrid[i,:]:
##                outfile.write('%s ' % str(value))
##            outfile.write('\n')
##        outfile.close()
    
    # create a new motion segment for registered motion segment
    registered_mo = RegisteredMotionSegment(mo1)
    #registered_mo.getRotationAngle(thetaGrid[0,0])
    registered_mo.getOrigin(mo2.data[0][:3])
    # create cost matrix
    # costMat = costMatrix(distGrid, DEBUG)
    # path = pathSearch(costMat, DEBUG)
    path = searchPath(distGrid, DEBUG)
    # find index of warpped motion
##    warppedVecIndex = []
##    for i in range(mo2.numFrames):
##        warppedVecIndex.append(max(xrange(len(path[:,i])), key = path[:,i].__getitem__))
    warppedVecIndex = getWarpIndex(path)
    alignmentCurve = getAlignmentCurve(thetaGrid, warppedVecIndex)
    registered_mo.getWarppingIndex(warppedVecIndex)
    registered_mo.getAlignmentCurve(alignmentCurve)
    # transform and rotate test motion
    registered_mo.transformation()
    #registered_mo.smooth()
    registered_mo.getMotion()
    return registered_mo

def getWarpIndex(path):
    warppedVecIndex = []
    m,n = path.shape
    keyvalue = max(path[:,0])
    for i in range(n):
        index = np.nonzero(path[:,i])[0][-1]
        warppedVecIndex.append(index)
    warppedVecIndex[0] = 0
    warppedVecIndex[-1] = int(m - 1)
    return warppedVecIndex

def getAlignmentCurve(thetaGrid, warppedVecIndex):
    num = len(warppedVecIndex)
    alignmentCurve = np.zeros(num)  # alignmentCurve contains transition parameters: rotation angle of each frame
    for i in range(num):
        alignmentCurve[i] = thetaGrid[warppedVecIndex[i],i]
    return alignmentCurve
  
def costMatrix(simMatrix, DEBUG = 0 ):
    # construct a cost matrix based on similarity matrix
    m,n = simMatrix.shape
    costMat = np.zeros([m+1, n+1])
    inf = float('inf')
    costMat[0,1:] = inf
    costMat[1:,0] = inf
    for i in range(m):
        for j in range(n):
            costMat[i+1, j+1] = simMatrix[i,j] + min([costMat[i,j], costMat[i+1,j], costMat[i,j+1]])
    if DEBUG:
        plt.imshow(costMat)
        plt.xlabel('reference motion')
        plt.ylabel('test motion')
        plt.show()
        # export cost matrix
        outfile = open('costMat.txt', 'w')
        m,n = costMat.shape
        for i in range(m):
            for value in costMat[i,:]:
                outfile.write('%s ' % str(value))
            outfile.write('\n')
        outfile.close()
        
    return costMat

def plotPath2D(worldpos):
    
    num = len(worldpos)
    points = []
    for i in range(num):
        newpoint = worldpos[i]['root'][0:2]
        points.append(newpoint)
    
    fig = plt.figure()
    plt.plot(*np.transpose(points))
    plt.show()
    #return points    
                                                        
def distMotions(mo1, mo2):
    dist = 0
    if mo1.numFrames != mo2.numFrames:
        #print 'warp motion 1 to motion 2'
        registered_mo1 = warpTwoMotion(mo1, mo2)
        plotPath2D(registered_mo1.worldpos)
        print 'alignment curve'
        print registered_mo1.alignmentCurve
        for i in range(mo2.numFrames):
            frameDist, rotation = distFrames(registered_mo1.worldpos[i], mo2.worldpos[i])
            dist += frameDist
    else:
        for i in range(mo2.numFrames):
            frameDist, rotation = distFrames(mo1.worldpos[i], mo2.worldpos[i])
            dist += frameDist
    dist = dist/mo2.numFrames
    return dist
        

