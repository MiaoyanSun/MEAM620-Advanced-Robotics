import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s



            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s




        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}

        # THE GAIN matrices  TO BE_determined

        #
        Kd = np.diag([5.3,5.4,5.5])
        #
        Kp = np.diag([8.4, 8.3, 8.6])
        #
        Kr = np.diag([2389, 2398, 399])
        #
        Kw = np.diag([59, 61, 58])


        # --------------- 1. Compute F_des from eqn (28, 27, 26) ----------------------------------

        # error of position ( = current position - current target position)
        err_position = np.array(state['x'] - flat_output['x'])
        # error of velocity ( = current velocity - current target velocity)
        err_velocity = np.array(state['v'] - flat_output['x_dot'])
        # r_des_dd is the desired acceleration
        r_des_dd = np.array(flat_output['x_ddot'] - Kd @ err_velocity - Kp @ err_position)
        #print(r_des_dd)
        # F_des is the "total commanded force including gracity"
        F_des = self.mass * r_des_dd + np.array([0,0,self.mass * self.g])

        # --------------- 2. Compute u1 from eqn (29) ---------------------------------------------

        # compute for b3
        # R is the rotation matrix, derived from the quaternion given
        R_state = Rotation.as_matrix(Rotation.from_quat(state['q']))
        #print(R_state)

        # b3 is the last column of R
        b_3 = np.array(R_state @ np.array([0,0,1]))

        u_1 = b_3.transpose() @ F_des


        # --------------- 3. determine R_des from eqn (33) and the definition of bi_des -----------

        acc_yaw = np.array([1, flat_output['yaw'], 0])

        b_3_des = np.array(F_des / np.linalg.norm(F_des))
        b_2_des = np.array((np.cross(b_3_des, acc_yaw)) / (np.linalg.norm(np.cross(b_3_des, acc_yaw))))
        b_1_des = np.cross(b_2_des, b_3_des)

        # here is R_des, the desired rotation matrix
        R_des = np.array([[b_1_des[0], b_2_des[0], b_3_des[0]],
                          [b_1_des[1], b_2_des[1], b_3_des[1]],
                          [b_1_des[2], b_2_des[2], b_3_des[2]]])

        # --------------- 4. find the error orientation error vector e_R from eqn (34) and substitute w for e_w ---

        e_R = 0.5 * ((R_des.T @ R_state) - (R_state.T @ R_des))

        e_R = np.array([-e_R[1][2], e_R[0][2], -e_R[0][1]]).reshape(3,1)  # 3 x 1

        e_w = np.array(state['w']).reshape(3,1)


        # --------------- 5. Compute u2 from eqn (35)

        u_2 = np.array(self.inertia @ (- Kr @ e_R - Kw @ e_w))


        # --------------- 6. Compute u from u1 and u2;   Find Fi

        u = np.vstack((u_1, u_2))
        #print(u)



        # --------------- 7. compute for "cmd_motor_speeds, motor speeds, rad/s, shape=(N,4)"

        gamma = self.k_drag / self.k_thrust
        #print(gamma)

        L = self.arm_length
        matrix_for_F_i = np.array([[1, 1, 1, 1],
                                   [0, L, 0, -L],
                                   [-L, 0, L, 0],
                                   [gamma, -gamma, gamma, -gamma]])

        F_i = np.linalg.inv(matrix_for_F_i) @ u

        #print(F_i)

        # F_i = k_F * w_i^2
        k_F = self.k_thrust # coff of force

        for i in range(0, 4):
            if F_i[i, 0] < 0:
                F_i[i, 0] = 0
                cmd_motor_speeds[i] = self.rotor_speed_max
            cmd_motor_speeds[i] = np.sqrt(F_i[i, 0] / self.k_thrust)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max


        cmd_thrust = u_1
        cmd_moment = u_2

      #  print(str(state['x']) + str(flat_output['x']))


        return control_input
