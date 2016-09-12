import sys
include_lib = r'../DTW/'
sys.path.append(include_lib)
from DTW import *
fileList1 = r'pose3\*.bvh'
fileList2 = r'pose3Topose2\*.bvh'
fileList = [fileList2, ]
for item in fileList:
    t = DTW(item)
    t.findReferenceMotion()
    print 'reference index: '+str(t.refIdx)
    t.warpMotionSegmentsToReferenceMotion()
    t.writeReferenceMotion()
