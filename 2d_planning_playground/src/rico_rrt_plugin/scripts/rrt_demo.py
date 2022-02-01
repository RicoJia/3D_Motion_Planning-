from utils import load_map
from rrt import RRT
import numpy as np
import cv2

if __name__=='__main__':
    mp = load_map()
    rrt = RRT(mp)
    rrt.plan()
    
