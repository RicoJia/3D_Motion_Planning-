#!/usr/bin/env python
'''
This file contains functions for adding obstacles to a numpy array plot.
Circular obstacles are drawn along the y axis. At each incremental position d along the y axis from the obstacle origin, a rectangular area is filled on the plot. The width of the
rectangle is cos(arcsin(d/r))
'''
import numpy as np
from PIL import Image
import cv2
from time import sleep

WINDOW_NAME = "RRT demo"

def generate_random_point(dim, upper_bounds, lower_bounds): 
    return np.random.uniform(np.array(lower_bounds), np.array(upper_bounds)).astype(int)

def draw_line(mp:np.ndarray, start, end): 
    # color = (0, 255, 0)
    color = 125
    thickness = 1
    image = cv2.line(mp, tuple(start), tuple(end), color, thickness)
    return image

def no_obstacle_in_line(mp: np.ndarray, start, end): 
    """
    Check for obstacles along a bresenham 8-connected line
    """
    zero_img = np.zeros_like(mp)
    zero_img = draw_line(zero_img, start, end)
    obstacles = np.logical_and(zero_img, mp)
    if np.count_nonzero(obstacles) == 0: 
        return True
    else: 
        return False
    
def is_within_boundary(row_num, column_num, pt):
    if pt[0]<row_num and pt[0]>=0:
        if pt[1]<column_num and pt[0]>=0:
            return True
    else:
        return False

def load_map():
    DARK_THRE = 100
    PIC_FILE_PATH = './obstacle2.jpg'
    img = cv2.imread(PIC_FILE_PATH, cv2.IMREAD_GRAYSCALE)
    # Caution: in this case we want white space to be free space
    _, img = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)
    return img

class WindowManager(object):
    """Managing visualize windows"""
    def __init__(self, canvas_map):
        cv2.namedWindow(WINDOW_NAME)
        self.canvas_map = np.copy(canvas_map)
    def __del__(self): 
        cv2.destroyAllWindows()
    def get_start_goal(self): 
        """
        Function that waits for two mouse clicks, which will be interpreted as start and goal
        """
        pts = []
        def click_pt(event, x_temp, y_temp, flags, params): 
            if event == cv2.EVENT_LBUTTONDOWN:
                pts.append(np.array([x_temp, y_temp]))

        cv2.setMouseCallback(WINDOW_NAME, click_pt) 

        while len(pts) < 2: 
            WindowManager.show_map(self.canvas_map, 10)
        return pts

    def show_new_edges(self, new_pt_pairs: list, wait_time = 100): 
        """
        Function for showing newly added points and their parents
        """
        for new_ptr_pair in new_pt_pairs: 
            draw_line(self.canvas_map, new_ptr_pair[0], new_ptr_pair[1])
        WindowManager.show_map(self.canvas_map, wait_time)

    @staticmethod
    def show_map(mp: np.ndarray, wait_time=0):
        cv2.imshow(WINDOW_NAME, mp)
        cv2.waitKey(wait_time)


# Kdtree
# Adopted from https://github.com/Vectorized/Python-KD-Tree/blob/master/kdtree.py
# Makes the KD-Tree for fast lookup: [[smaller half], [larger half], root]
def dist(p1, p2): 
    return np.linalg.norm(np.array(p1) - np.array(p2))

def make_kd_tree(points, dim, i=0):
    if len(points) > 1:
        points.sort(key=lambda x: x[i])
        i = (i + 1) % dim
        half = len(points) >> 1
        return [
            make_kd_tree(points[: half], dim, i),
            make_kd_tree(points[half + 1:], dim, i),
            points[half]
        ]
    elif len(points) == 1:
        return [None, None, points[0]]

# Adds a point to the kd-tree
def add_point(kd_node, point, dim, i=0):
    if kd_node is not None:
        dx = kd_node[2][i] - point[i]
        i = (i + 1) % dim
        # if dx < 0, go left. 
        for j, c in ((0, dx >= 0), (1, dx < 0)):
            if c and kd_node[j] is None:
                kd_node[j] = [None, None, point]
            elif c:
                add_point(kd_node[j], point, dim, i)

# k nearest neighbors
def get_knn(kd_node, point, k, dim, dist_func = dist, return_distances=True, i=0, heap=None):
    import heapq
    is_root = not heap
    if is_root:
        heap = []
    if kd_node is not None:
        dist = dist_func(point, kd_node[2])
        dx = kd_node[2][i] - point[i]
        if len(heap) < k:
            heapq.heappush(heap, (-dist, kd_node[2]))
        elif dist < -heap[0][0]:
            heapq.heappushpop(heap, (-dist, kd_node[2]))
        i = (i + 1) % dim
        # Goes into the left branch, and then the right branch if needed
        for b in [dx < 0] + [dx >= 0] * (dx * dx < -heap[0][0]):
            get_knn(kd_node[b], point, k, dim, dist_func, return_distances, i, heap)
    if is_root:
        neighbors = sorted((-h[0], h[1]) for h in heap)
        return neighbors if return_distances else [n[1] for n in neighbors]


# Simple and efficient implementation: 1. search the area that contains the target 2. Search the rest of the area. Quit once the the best possible (dx * dx) is greater than the existing best.
def get_nearest(kd_node, point, dim, dist_func=dist, return_distances=True, i=0, best=None):
    if kd_node is not None:
        dist = dist_func(point, kd_node[2])
        dx = kd_node[2][i] - point[i]
        if not best:
            best = [dist, kd_node[2]]
        elif dist < best[0]:
            best[0], best[1] = dist, kd_node[2]
        i = (i + 1) % dim
        # Goes into the left/right branch (depending on dx), and then the other branch if needed
        # Rico: Smart. 
        for b in [dx < 0] + [dx >= 0] * (dx * dx < best[0]):
            get_nearest(kd_node[b], point, dim, dist_func, return_distances, i, best)
    return best if return_distances else best[1]


if __name__ == "__main__": 
    # kd tree test 
    points = [[1,0,0], [0,1,0], [0,0,1]]
    kd_tree = make_kd_tree(points, dim = len(points[0]))
    print(get_nearest(kd_tree, [0.2, 0, 0], dim = len(points[0])))
    add_point(kd_tree, [0.2, 0, 0], dim = 3)
    print(get_nearest(kd_tree, [0.5, 0, 0], dim = len(points[0])))

