import sys
#include_path = r'K:\random walking synthesis_python script\random walking synthesis\GMM'
include_path = r'GMM/'
sys.path.append(include_path)
print sys.path
#sfrom MorphableModel import MorphabelModel
from PCA import PCA
#from test1 import test
import numpy as np

DATA = 1 
p = PCA()
