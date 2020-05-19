"""
This script provides examples of how to retrieve and plot data from laboratory.

You will need to improve these plots for use in your laboratory report. For
example, you should only plot data from the interesting portion of the
trajectory. You may also want to create plots not provided here.

The sandbox scripts from earlier labs provide examples of computing and plotting
your waypoints and trajectory functions.
"""

from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation as R

from flightsim.axes3ds import Axes3Ds
from flightsim.world import World

# Put the path to your bagfile here! It is easiest to put the bag files in the
# same directory you run the script in.
bagfile = 'lab1.bag'

# Load the flight data from bag.
state = {}
control = {}
with rosbag.Bag(bagfile) as bag:
    odometry = np.array([
        np.array([t.to_sec() - bag.get_start_time(),
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z,
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            for (_, msg, t) in bag.read_messages(topics=['odom'])])
    vicon_time = odometry[:, 0]
    state['x'] = odometry[:, 1:4]
    state['v'] = odometry[:, 4:7]
    state['w'] = odometry[:, 7:10]
    state['q'] = odometry[:, 10:15]

    commands = np.array([
        np.array([t.to_sec() - bag.get_start_time(),
        msg.linear.z, msg.linear.y, msg.linear.x])
            for (_, msg, t) in bag.read_messages(topics=['so3cmd_to_crazyflie/cmd_vel_fast'])])
    command_time = commands[:, 0]
    c1 = -0.6709 # Coefficients to convert thrust PWM to Newtons.
    c2 = 0.1932
    c3 = 13.0652
    control['cmd_thrust'] = (((commands[:, 1]/60000 - c1) / c2)**2 - c3)/1000*9.81
    control['cmd_q'] = R.from_euler('zyx', np.transpose([commands[:, 2], commands[:, 3],
                                                         np.zeros(commands[:, 2].shape)]), degrees=True).as_quat()
# Position and Velocity vs. Time
(fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Position vs Time')
x = state['x']
ax = axes[0]
ax.plot(vicon_time, x[:, 0], 'r.',    vicon_time, x[:, 1], 'g.',    vicon_time, x[:, 2], 'b.')
ax.legend(('x', 'y', 'z'), loc='upper right')
ax.set_ylabel('position, m')
ax.grid('major')
ax.set_title('Position')
v = state['v']
ax = axes[1]
ax.plot(vicon_time, v[:, 0], 'r.',    vicon_time, v[:, 1], 'g.',    vicon_time, v[:, 2], 'b.')
ax.legend(('x', 'y', 'z'), loc='upper right')
ax.set_ylabel('velocity, m/s')
ax.set_xlabel('vicon_time, s')
ax.grid('major')

# 3D Position
# Either load a world definition file with boundaries and obstacles like this:
# world = World.from_file('test_lab_1.json')
# Or define an empty world and set the boundary size like this:
world = World.empty([-2, 2, -2, 2, 0, 4])

fig = plt.figure('3D Path')
ax = Axes3Ds(fig)
world.draw(ax)
world.draw_points(ax, state['x'], color='blue', markersize=4)
ax.legend(handles=[
    Line2D([], [], color='blue', linestyle='', marker='.', markersize=4, label='Flight')],
    loc='upper right')

# Commands vs. Time
(fig, axes) = plt.subplots(nrows=1, ncols=1, sharex=True, num='Commands vs Time')
T = control['cmd_thrust']
ax = axes
ax.plot(command_time, T, 'k.')
ax.set_ylabel('thrust, N')
ax.set_xlabel('time, s')
ax.grid('major')

# Orientation and Angular Velocity vs. Time
(fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Orientation vs Time')
q_des = control['cmd_q']
q = state['q']
ax = axes[0]
ax.plot(command_time, q_des[:, 0], 'r', command_time, q_des[:, 1], 'g',
        command_time, q_des[:, 2], 'b', command_time, q_des[:, 3], 'k')
ax.plot(vicon_time, q[:, 0], 'r.',    vicon_time, q[:, 1], 'g.',
        vicon_time, q[:, 2], 'b.',    vicon_time, q[:, 3],     'k.')
ax.legend(('i', 'j', 'k', 'w'), loc='upper right')
ax.set_ylabel('quaternion')
ax.set_xlabel('time, s')
ax.grid('major')
w = state['w']
ax = axes[1]
ax.plot(vicon_time, w[:, 0], 'r.', vicon_time, w[:, 1], 'g.', vicon_time, w[:, 2], 'b.')
ax.legend(('x', 'y', 'z'), loc='upper right')
ax.set_ylabel('angular velocity, rad/s')
ax.set_xlabel('time, s')
ax.grid('major')

plt.show()
