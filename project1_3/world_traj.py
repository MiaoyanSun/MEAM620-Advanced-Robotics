
import numpy as np

from proj1_3.code.graph_search import graph_search


class WorldTraj(object):

    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """
        self.resolution = np.array([0.25, 0.25, 0.25])  # resolution, the discretization of the occupancy grid in x,y,z[0.25, 0.25, 0.25]
        self.margin = 0.25
        self.velocity = 1.6  # -------------------------------------------------============================
        # margin, the inflation radius used to create the configuration space (assuming a spherical drone)
        self.path = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        # ------------------------ Reduce Points-----------------------------------
        self.points = np.zeros((1, 3))

        # my path:
        # make a copy of the path returned from graph_search()
        my_path = self.path

        # x/x\x/x/x-x-x-x path returned from graph_search is dense, nodes are next to each other so it would be meaningless
        # to cut down points based on interval distance between nodes for now.

        # reduce points that are on the same straight line
        my_path_1 = []
        vector = np.array(my_path[1] - my_path[0])
        my_path_1.append(my_path[0])
        my_path_1.append(my_path[1])
        u_prev = vector / np.linalg.norm(vector)   # unit vector
        mark = 1
        for i in range (1, my_path.shape[0]-5):
            vector_curr = my_path[i+1] - my_path[i]
            u_curr = vector_curr / np.linalg.norm(vector_curr)
            if np.array_equal(u_prev, u_curr):
                if np.linalg.norm(my_path[i+1] - my_path[mark]) > 1: # two points are too far away, not good must add
                    my_path_1.append(my_path[i+1])
                    mark = i+1
                # same means 3 points are in the same line, don't add i+1
                u_prev = u_curr
            else:
                # not same means 3 points not in the same line, add i+1
                my_path_1.append(my_path[i+1])
                u_prev = u_curr
            # may or may not add the goal point

        my_path_1.append(my_path[-1])
        self.points = np.asarray(my_path_1)


# time starts
        # STUDENT CODE HERE

        self.time_interval = [0]
        self.time_accumulated = [0]
        self.time_total = 0

        for i in range (0, self.points.shape[0]-1):
            distance = np.linalg.norm(self.points[i+1] - self.points[i])
            self.time_interval.append(distance/self.velocity)
            self.time_accumulated.append(self.time_accumulated[i] + self.time_interval[i+1])
        self.time_total = np.sum(self.time_interval)

# time ends
# A starts
        # -------------- Min Jerk Spline -----------------***

        # step 5: construct A matrix for Ax = b
        matrix_size = 6 * (len(self.points) - 1) # unknowns = 6 * segments

        A = np.zeros([matrix_size, matrix_size])
        b = np.zeros([matrix_size, 3])
        b[0] = self.points[0]
        b[-3] = self.points[-1]

        # Reference: Siyan Wang taught me how to replace submatrices in big matrix through constraining its column and row "A[r1:r3, c1:c3]"
        # This has greatly cleaned up my code.

        # Boundary condition of first point
        A[0:3, 0:6] = [[0,0,0,0,0,1],
                       [0,0,0,0,1,0],
                       [0,0,0,2,0,0]]
        # Boundary condition of last point
        t_ = self.time_interval[-1]
        A[-3:, -6:] = [[t_**5, t_**4, t_**3, t_**2, t_, 1],
                       [5*t_**4, 4*t_**3, 3*t_**2, 2*t_, 1, 0],
                       [20*t_**3, 12*t_**2, 6*t_, 2, 0, 0]]

        t = self.time_interval
        size6 = len(self.points)
        for i in range(1, size6- 1):
            A[6*i-3:6*i+3, 6*i-6:6*i+6] = [[t[i]**5, t[i]**4, t[i]**3, t[i]**2, t[i],1,0,0,0,0,0,0],
                                           [0,0,0,0,0,0,0,0,0,0,0,1],
                                           [5*t[i]**4, 4*t[i]**3, 3*t[i]**2, 2*t[i],1,0,0,0,0,0,-1,0],
                                           [20*t[i]**3, 12*t[i]**2, 6*t[i], 2, 0, 0, 0, 0, 0, -2, 0, 0],
                                           [60*t[i]**2, 24*t[i],6,0,0,0,0,0,-6,0,0,0],
                                           [120*t[i],24,0,0,0,0,0,-24,0,0,0,0]]

            b[6*i-3] = self.points[i]
            b[6*i-2] = self.points[i]

        self.Coef = np.linalg.solve(A, b)




    def update(self, t):

        x = np.zeros((3,))  # Position
        x_dot = np.zeros((3,))  # Velocity
        x_ddot = np.zeros((3,))  # Acceleration
        x_dddot = np.zeros((3,))  # Jerk
        x_ddddot = np.zeros((3,))  # Snap
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        if t > self.time_total:
            x = self.points[-1]
            x_dot = np.zeros((3,))
            x_ddot = np.zeros((3,))
            flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot, 'yaw': yaw, 'yaw_dot': yaw_dot}
            return flat_output


        for i in range(len(self.time_accumulated) - 1):
            if t < self.time_accumulated[i + 1]:
                dt = t - self.time_accumulated[i]
                x = np.array([dt ** 5, dt ** 4, dt ** 3, dt ** 2, dt ** 1, 1]) @ self.Coef[6 * i:6 * i + 6]
                x_dot = np.array([5 * dt ** 4, 4 * dt ** 3, 3 * dt ** 2, 2 * dt, 1, 0]) @ self.Coef[6 * i:6 * i + 6]
                x_ddot = np.array([20 * dt ** 3, 12 * dt ** 2, 6 * dt, 2, 0, 0]) @ self.Coef[6 * i:6 * i + 6]
                x_dddot = np.array([60 * dt ** 2, 24 * dt, 6, 0, 0, 0]) @ self.Coef[6 * i:6 * i + 6]
                x_ddddot = np.array([120 * dt, 24, 0, 0, 0, 0]) @ self.Coef[6 * i:6 * i + 6]
                break


        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                                   'yaw': yaw, 'yaw_dot': yaw_dot}


        return flat_output
