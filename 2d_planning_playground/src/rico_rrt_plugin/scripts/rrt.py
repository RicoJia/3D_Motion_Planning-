import numpy as np
from utils import draw_line, WindowManager, make_kd_tree, add_point, get_nearest, generate_random_point, no_obstacle_in_line

class RRT():
    """Vanilla RRT"""
    def __init__(self, mp):
        """
        __init__ialize map, and get start and goal out of the window_mgr
        """        
        self.mp = mp
        self.window_mgr = WindowManager(self.mp)
        self.start, self.goal = self.window_mgr.get_start_goal()

        if self.mp[tuple(self.start[::-1].astype(int))] != 0 or self.mp[tuple(self.goal[::-1].astype(int))] != 0: 
            raise RuntimeError("Invalid start or goal")

        self.dim = len(self.start)
        self.lower_bounds = np.zeros(self.dim)
        self.upper_bounds = self.mp.shape[0:2]
        # {coordinates: parent_coordinates}
        self.edge_len = 10.0
        self.lookup = {self.start.tobytes(): None}
        self.build_kd_tree()
        print(f"__init__ info: start - {self.start}, goal - {self.goal}")

    def plan(self): 
        """
        Main planning function for RRT
        """
        if not no_obstacle_in_line(self.mp, self.start, self.goal): 
            while True: 
                new_pt_pairs = self.plan_one_iteration()
                self.window_mgr.show_new_edges(new_pt_pairs, 1)
                if new_pt_pairs and no_obstacle_in_line(self.mp, new_pt_pairs[0][1], self.goal): 
                    self.lookup[self.goal.tobytes()] = new_pt_pairs[0][1]
                    break
        else: 
            self.add_point_to_rrt(self.start, self.goal)
        self.window_mgr.show_new_edges(self.backtrack(self.goal), 10000)

    def build_kd_tree(self): 
        self.kd_tree = make_kd_tree([self.start], len(self.start))

    def get_valid_new_node_pair(self, random_pt, kd_tree): 
        """
        Return [nearest_node, new_node]. Empty if new_node -> random_pt has obstacle
        """
        dist, nearest_node = get_nearest(kd_tree, random_pt, self.dim)
        if dist > self.edge_len: 
            random_pt = (nearest_node + (random_pt - nearest_node) * self.edge_len/dist).astype(int)
        if no_obstacle_in_line(self.mp, nearest_node, random_pt):
            return [nearest_node, random_pt]
        else: 
            return []

    def add_point_to_rrt(self, nearest_node, random_pt, kd_tree): 
        """
        Add random_pt to kd_tree and lookup if no obstacle to nearest_node
        """
        add_point(kd_tree, random_pt, self.dim)
        self.lookup[random_pt.tobytes()] = nearest_node

    def plan_one_iteration(self): 
        random_pt = generate_random_point(self.dim, self.lower_bounds, self.upper_bounds)
        new_pt_pair = self.get_valid_new_node_pair(random_pt, self.kd_tree)
        if new_pt_pair: 
            self.add_point_to_rrt(new_pt_pair[0], new_pt_pair[1], self.kd_tree)
            return [new_pt_pair]
        else: 
            return []

    def backtrack(self, start):
        """
        Back tracking function for all RRT methods
        """
        node = start
        path = []
        while True:
            next_node = self.lookup[node.tobytes()]
            if next_node is None:
                break
            path.append([node, next_node])
            node = next_node
        return path

class RRTConnect(RRT):
    """RRT Connect Vanilla"""
    def __init__(self, mp):
        super(RRTConnect, self).__init__(mp)
        self.lookup[self.goal.tobytes()] = None

    def build_kd_tree(self): 
        self.kd_tree_start = make_kd_tree([self.start], len(self.start))
        self.kd_tree_goal = make_kd_tree([self.goal], len(self.goal))
        
    def plan(self):
        """
        Main planning function for RRTConnect
        """
        bridge = []
        if not no_obstacle_in_line(self.mp, self.start, self.goal):
            while True:
                new_pt_pairs = self.plan_one_iteration()
                self.window_mgr.show_new_edges(new_pt_pairs, 1)
                if new_pt_pairs and no_obstacle_in_line(self.mp, new_pt_pairs[0][1], new_pt_pairs[1][1]):
                    bridge = [new_pt_pairs[0][1], new_pt_pairs[1][1]]
                    break
        else:
            bridge = [self.start, self.goal]
        path = [bridge]
        path+=(self.backtrack(bridge[0]))
        path+=(self.backtrack(bridge[1]))
        self.window_mgr.show_new_edges(path, 10000, final_show = True)

    def plan_one_iteration(self): 
        random_pt = generate_random_point(self.dim, self.lower_bounds, self.upper_bounds)
        new_pt_pair_s = self.get_valid_new_node_pair(random_pt, self.kd_tree_start)
        new_pt_pair_g = self.get_valid_new_node_pair(random_pt, self.kd_tree_goal)
        if new_pt_pair_s and new_pt_pair_g:
            self.add_point_to_rrt(new_pt_pair_s[0], new_pt_pair_s[1], self.kd_tree_start)
            self.add_point_to_rrt(new_pt_pair_g[0], new_pt_pair_g[1], self.kd_tree_goal)
            return [new_pt_pair_s, new_pt_pair_g]
        else:
            return []
