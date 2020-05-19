import numpy as np



class Node:
    def __init__(self, index_coordinates):
        self.index_coordinate = index_coordinates
        self.k = np.inf
        self.g = np.inf
        self.h = np.inf
        self.parent = None
        #self.neighbors = []


    # def get_g(self):
    #     return self.g
    #
    # def get_k(self):
    #     return self.k
    #
    # def get_h(self):
    #     return self.h

    # def set_g(self, g):
    #     self.g = g
    #     return

    def set_k(self, k = None):
        if k is not None:
            self.k = k
        else:
            self.k = self.g + self.h
        return

    # def set_parent(self, node):
    #     self.parent = node
    #     return

    # def get_parent(self):
    #     return self.parent

    # def add_neighbors(self, node):
    #     self.neighbors.append(node)
    #     return

    def get_index_coordinates(self):
        return self.index_coordinate

    # def get_neighbors(self):
    #     return self.neighbors
