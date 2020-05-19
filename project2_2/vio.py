#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE

    # p : current position
    # v : current velocity
    # q : current rotation <Rotation>
    # a_b : accelerometer bias
    # w_b : gyroscope bias
    # g : current estimate for gravity vector

    # new position:
    Rot = q.as_matrix()
    new_p = p + v * dt + (1/2) * (Rot@(a_m - a_b) + g) * dt**2

    # new velocity:
    new_v = v + (Rot@(a_m - a_b) + g) * dt

    # new rotation: post multiply
    new_q = q * Rotation.from_rotvec(((w_m - w_b) * dt).reshape(3))


    return new_p, new_v, new_q, a_b, w_b, g









def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE
    #cov_matrix = np.identity(18)

    Rot = q.as_matrix()
    
    Fx = np.identity(18)
    Fi = np.zeros((18,12))
    Qi = np.zeros((12,12))

    # compute for Fx:
    Fx[0:3, 3:6] = np.identity(3) * dt
    a_skew = np.zeros((3,3))
    a = a_m - a_b
    a_skew = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    Fx[3:6, 6:9] = - Rot @ a_skew * dt
    Fx[3:6, 9:12] = - Rot * dt
    Fx[3:6, 15:18] = np.identity(3) * dt
                                      
    #Fx[6:9, 6:9] = Rot @ np.transpose(((Rotation.from_rotvec(((w_m - w_b) * dt).reshape(3))).as_matrix()).reshape(3, 3)) #######/////////////
    Fx[6:9, 6:9] =       np.transpose(((Rotation.from_rotvec(((w_m - w_b) * dt).reshape((1,3))).as_matrix()).reshape(3, 3)))

    Fx[6:9, 12:15] = - np.identity(3) * dt


    # compute for Fi
    Fi[3:6, 0:3] = np.identity(3)
    Fi[6:9, 3:6] = np.identity(3)
    Fi[9:12, 6:9] = np.identity(3)
    Fi[12:15, 9:12] = np.identity(3)


    # compute for Qi
    V_i    = (accelerometer_noise_density ** 2) * (dt ** 2) * np.identity(3)
    the_i  = (gyroscope_noise_density ** 2) * (dt ** 2) * np.identity(3)
    A_i    = (accelerometer_random_walk ** 2) * dt * np.identity(3)
    omg_i  = (gyroscope_random_walk ** 2) * dt * np.identity(3)

    Qi[0:3, 0:3]    = V_i
    Qi[3:6, 3:6]    = the_i
    Qi[6:9, 6:9]    = A_i
    Qi[9:12, 9:12]  = omg_i
    

    # update error_state_covariance   P = Fx P Fx_T  +  Fi Qi Fi_t
    error_state_covariance = Fx @ error_state_covariance @ Fx.T + Fi @ Qi @ Fi.T

    # return an 18x18 covariance matrix
    return error_state_covariance


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """
    
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE - compute the innovation next state, next error_state covariance

    H = np.zeros((2,18))
    
    Rot = q.as_matrix()
    print("shape Rot", Rot.shape)
    print("shape Pw, p", Pw.shape, p.shape)
    Pc = Rot.T @ (Pw - p)
    
    predicted_uv = np.array([Pc[0], Pc[1]]).reshape(2, 1) / Pc[2]
    
     
    #predicted_uv = np.array([[Pc[0]/Pc[2]],[Pc[1]/Pc[2]]]).reshape((2, 1))
    
    innovation = uv - predicted_uv
    error = norm(innovation)
    
    if error < error_threshold:
    
        dZ_dPc = np.array([[1, 0, -predicted_uv[0]], [0, 1, -predicted_uv[1]]]) / Pc[2]
    
        dPc_dThe = np.array([[0, -Pc[2], Pc[1]],
                              [Pc[2], 0, -Pc[0]],
                              [-Pc[1], Pc[0], 0]])
    
        H[0:3, 0:3] = dZ_dPc @ - Rot.T
        H[0:3, 6:9] = dZ_dPc @ dPc_dThe
    
         # karman gain
        Kt = error_state_covariance @ H.T @ inv(H @ error_state_covariance @ H.T + Q)
    
        del_x = Kt @ innovation
        p += del_x[0:3]
        v += del_x[3:6]
        q = q * Rotation.from_rotvec(del_x[6:9].reshape(3))
        a_b += del_x[9:12]
        w_b += del_x[12:15]
        g += del_x[15:18]
    
        error_state_covariance = (np.identity(18) - Kt @ H) @ error_state_covariance @ (np.identity(18) - Kt @ H).T + Kt @ Q @ Kt.T

    return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
