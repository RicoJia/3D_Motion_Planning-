import numpy as np
from utils import draw_line, WindowManager, make_kd_tree, add_point, get_nearest, generate_random_point, no_obstacle_in_line

class RRTBasis:
    """Base class of RRT"""
    def __init__(self, mp):
        """
        Initialize map, and get start and goal out of the window_mgr
        """
        self.mp = mp
        self.window_mgr = WindowManager(self.mp)
        self.start, self.goal = self.window_mgr.get_start_goal()

        #TODO 
        if self.mp[tuple(self.start[::-1].astype(int))] != 0 or self.mp[tuple(self.goal[::-1].astype(int))] != 0: 
            raise RuntimeError("Invalid start or goal")

        self.dim = len(self.start)
        self.lower_bounds = np.zeros(self.dim)
        self.upper_bounds = self.mp.shape[0:2]
        # {coordinates: parent_coordinates}
        self.lookup = dict()
        self.edge_len = 10.0
        print(f"init info: start - {self.start}, goal - {self.goal}")

    def plan(self): 
        pass

    def __plan_one_iteration(self): 
        pass

    def backtrack(self): 
        """
        Back tracking function for all RRT methods
        """
        node = self.goal
        path = []
        while True: 
            next_node = self.lookup[node.tobytes()]
            if next_node is None: 
                break
            path.append([node, next_node])
            node = next_node
        return path

class RRT(RRTBasis):
    """Vanilla RRT"""
    def __init__(self, mp):
        super(RRT, self).__init__(mp)

    def plan(self): 
        """
        Main planning function for RRT
        """
        self.kd_tree = make_kd_tree([self.start], len(self.start))
        self.lookup[self.start.tobytes()] = None
        if not self.__add_point_if_no_obstacle(self.start, self.goal):
            while True: 
                new_pt_pairs = self.__plan_one_iteration()
                self.window_mgr.show_new_edges(new_pt_pairs, 1)
                if new_pt_pairs and self.__solution_found(new_pt_pairs[0][1]): 
                    self.lookup[self.goal.tobytes()] = new_pt_pairs[0][1]
                    break
        self.window_mgr.show_new_edges(self.backtrack(), 10000)

    def __solution_found(self, node): 
        return True if no_obstacle_in_line(self.mp, node, self.goal) else False

    def __add_point_if_no_obstacle(self, nearest_node, random_pt): 
        """
        Add random_pt to kd_tree and lookup if no obstacle to nearest_node
        """
        if no_obstacle_in_line(self.mp, nearest_node, random_pt): 
            add_point(self.kd_tree, random_pt, self.dim)
            self.lookup[random_pt.tobytes()] = nearest_node
            return True
        else: 
            return False

    def __plan_one_iteration(self): 
        random_pt = generate_random_point(self.dim, self.lower_bounds, self.upper_bounds)
        dist, nearest_node = get_nearest(self.kd_tree, random_pt, self.dim)
        if dist > self.edge_len: 
            random_pt = (nearest_node + (random_pt - nearest_node) * self.edge_len/dist).astype(int)
        if self.__add_point_if_no_obstacle(nearest_node, random_pt):
            new_pt_pairs = [[nearest_node, random_pt]]
        else: 
            new_pt_pairs = []
        return new_pt_pairs
        
