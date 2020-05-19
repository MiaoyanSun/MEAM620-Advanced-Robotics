import numpy as np

class WaypointTraj(object):
    """

    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE

        self.points = points
        print(self.points)
        l = len(points)
        self.point_time_est = [0.0] * l

        self.speed = 1.5    # m/s
        self.velocity = [np.zeros((3,))] * l
        self.distance = [np.zeros((3,))] * l
        self.displacement_unit_vector = [np.zeros((3,))] * l



        if l > 1: # there are more than 1 points to be reached
            for i in range(1, l):
                self.distance[i] = np.linalg.norm(np.array(self.points[i] - self.points[i - 1]))
                self.displacement_unit_vector[i] = np.array(self.points[i] - self.points[i - 1]) / self.distance[i]
                self.velocity[i] = self.speed * self.displacement_unit_vector[i]
                self.point_time_est[i] = self.point_time_est[i - 1] + self.distance[i] / self.speed  # time is accumulative

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        if len(self.points) > 1:
            if t == 0:    # allow for the quad to reach the first point if points[0] != initial state
                x = self.points[0]
                x_dot = np.zeros((3,))

            elif t > self.point_time_est[-1]:
                x = self.points[-1]
                x_dot = np.zeros((3,))

            else:
                for i in range (1, len(self.points)):
                    if t <= self.point_time_est[i]:
                        x = self.points[i]
                        x_dot = self.velocity[i]  # give it the exact velocity pushes it too much, a factor of * mitigate the effect
                        break
        else:
            x = self.points[0]
            x_dot = np.zeros((3,))



        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output





