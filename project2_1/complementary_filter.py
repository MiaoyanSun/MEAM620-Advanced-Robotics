# %% Imports

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as R


# %%

def complementary_filter_update(initial_rotation, angular_velocity, linear_acceleration, dt):
    """
    Implements a complementary filter update

    :param initial_rotation: rotation_estimate at start of update
    :param angular_velocity: angular velocity vector at start of interval in radians per second
    :param linear_acceleration: linear acceleration vector at end of interval in meters per second squared
    :param dt: duration of interval in seconds
    :return: final_rotation - rotation estimate after update
    """

    #TODO Your code here - replace the return value with one you compute

    linear_acceleration = linear_acceleration / 9.81
    # 1: calculate error:  em = abs(acc - 1)
    error = abs(np.linalg.norm(linear_acceleration) - 1)

    # 2: gain
    if error < 0.1:
        alpha = 1
    elif error > 0.2:
        alpha = 0
    else:
        alpha = 2 - 10 * error

    # 3: get angular velocity w, and write skew symmetric form
    w1, w2, w3 = angular_velocity[0], angular_velocity[1], angular_velocity[2]
    w_skew = np.array([[0, -w3, w2], [w3, 0, -w1], [-w2, w1, 0]]) # skew omega

    # 4: R12 = exp(w^ dt)
    step_rotation = R.from_matrix(np.exp(w_skew * dt))
    new_rotation = initial_rotation * step_rotation
    new_rotation = new_rotation.as_matrix()

    # 5: g' = R * acc
    g_prime = np.matmul(new_rotation, linear_acceleration)
    g_prime_norm = g_prime / np.linalg.norm(g_prime)

    # 6: q_acc
    g_px, g_py, g_pz = g_prime_norm[0], g_prime_norm[1], g_prime_norm[2]

    q_acc = np.array([0, g_pz/(2*(g_px + 1))**0.5, -g_py/(2*(g_px + 1))**0.5, ((g_px+1)*0.5)**0.5])

    # 7: q_acc_prime
    qI = np.array([0,0,0,1])
    q_acc_prime = (1 - alpha)*qI + alpha * q_acc
    return R.from_quat(q_acc_prime)*R.from_matrix(new_rotation)



