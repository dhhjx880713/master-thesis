import sys
include_path = r'motion synthesis/'
sys.path.append(include_path)
from randomWalking import *
def main():
    try:
        input_num = int(raw_input('Please input the steps number : '))
    except ValueError:
        print "Not a number"
    w = randomWalking(input_num)

if __name__ == "__main__":
    main()
