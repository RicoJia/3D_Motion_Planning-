from utils import load_map
from rrt import RRT, RRTConnect
import numpy as np
import cv2

if __name__=='__main__':
    mp = load_map()
    rrt_connect = RRTConnect(mp)
    rrt_connect.plan()
    # rrt = RRT(mp)
    # rrt.plan()
    
