# Imports

import numpy as np
from scipy.spatial.transform import Rotation


# %%

def estimate_pose(uvd1, uvd2, pose_iterations, ransac_iterations, ransac_threshold):
    """
    Estimate Pose by repeatedly calling ransac

    :param uvd1:
    :param uvd2:
    :param pose_iterations:
    :param ransac_iterations:
    :param ransac_threshold:
    :return: Rotation, R; Translation, T; inliers, array of n booleans
    """

    R = Rotation.identity()

    for i in range(0, pose_iterations):
        w, t, inliers = ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold)
        R = Rotation.from_rotvec(w.ravel()) * R

    return R, t, inliers


def ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold):
    # find total number of correspondences
    n = uvd1.shape[1]

    # initialize inliers all false
    best_inliers = np.zeros(n, dtype=bool)

    for i in range(0, ransac_iterations):
        # Select 3  correspondences
        selection = np.random.choice(n, 3, replace=False)

        # Solve for w and  t
        w, t = solve_w_t(uvd1[:, selection], uvd2[:, selection], R)

        # find inliers
        inliers = find_inliers(w, t, uvd1, uvd2, R, ransac_threshold)

        # Update best inliers
        if inliers.sum() > best_inliers.sum():
            best_inliers = inliers.copy()

    # Solve for w and t using best inliers
    w, t = solve_w_t(uvd1[:, best_inliers], uvd2[:, best_inliers], R)

    return w, t, find_inliers(w, t, uvd1, uvd2, R, ransac_threshold)



def find_inliers(w, t, uvd1, uvd2, R0, threshold):
    """

    find_inliers core routine used to detect which correspondences are inliers

    :param w: ndarray with 3 entries angular velocity vector in radians/sec
    :param t: ndarray with 3 entries, translation vector
    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2:  3xn ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :param threshold: Threshold to use
    :return: ndarray with n boolean entries : Only True for correspondences that pass the test
    """


    n = uvd1.shape[1]  # column

    # TODO Your code here replace the dummy return value with a value you compute

    result = np.zeros(n, dtype='bool') # 1 for inlier and 0 for outlier
    w_t = np.zeros((6,1))
    w_t[0],w_t[1],w_t[2],w_t[3],w_t[4],w_t[5] = w[0],w[1],w[2],t[0],t[1],t[2]
    #w_t.reshape(6,1)

    for col in range(n):
        u1p, v1p = uvd1[0, col], uvd1[1, col]
        u2p, v2p = uvd2[0, col], uvd2[1, col]
        d2p = uvd2[2, col]

        y = R0.as_matrix() @ np.array([u2p, v2p, 1])  # matrix
        y1, y2, y3 = y[0], y[1], y[2]

        b_temp = (-np.array([[1, 0, -u1p], [0, 1, -v1p]]) @ y).reshape(2, 1)

        matrix1 = np.array([[1, 0, -u1p], [0, 1, -v1p]])
        matrix2 = np.array([[0, y3, -y2, d2p, 0, 0], [-y3, 0, y1, 0, d2p, 0], [y2, -y1, 0, 0, 0, d2p]])
        combine_matrix = np.matmul(matrix1, matrix2).reshape(2, 6)

        difference = np.linalg.norm(np.matmul(combine_matrix,w_t) - b_temp)

        if difference < threshold:
            result[col] = True
        else:
            continue
    return result

def solve_w_t(uvd1, uvd2, R0):
    """
    solve_w_t core routine used to compute best fit w and t given a set of stereo correspondences

    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
    :param R0: Rotation type - base rotation estimate
    :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
    """

    # TODO Your code here replace the dummy return value with a value you compute

    w = t = np.zeros((3, 1))
    columns = uvd1.shape[1]
    A = np.empty((columns*2, 6))  # Ax = b solve this matrix
    b = np.empty((columns*2, 1))

    for col in range(columns):
        u1p, v1p = uvd1[0, col], uvd1[1, col]
        u2p, v2p = uvd2[0, col], uvd2[1, col]
        d2p      = uvd2[2, col]

        # compute y matrix
        y = R0.as_matrix() @ np.array([u2p, v2p, 1])  # matrix
        y1, y2, y3 = y[0], y[1], y[2]

        b_temp = (-np.array([[1, 0, -u1p], [0, 1, -v1p]]) @ y).reshape(2,1)
        for i in range (0,2):
            b[2*col + i][0] = b_temp[i][0]

        matrix1 = np.array([[1, 0, -u1p], [0, 1, -v1p]])
        matrix2 = np.array([[0, y3, -y2, d2p, 0, 0], [-y3, 0, y1, 0, d2p, 0], [y2, -y1, 0, 0, 0, d2p]])
        combine_matrix = np.matmul(matrix1, matrix2).reshape(2,6)

        for i in range (0,2):
            for j in range (0,6):
                A[i+2*col][j] = combine_matrix[i][j]

    result = np.linalg.lstsq(A, b,rcond=None)[0]
    w,t = result[0:3], result[3:6]
    return w, t
