import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass

kPlinear = 1.0
kPangular = 1.0

@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # in radians

def rotation_matrix_2d(theta_deg):
    theta = np.radians(theta_deg)
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

body_width = 3
body_height = 2
s = 3  # distance between wheels
wheel_radius = 0.5
wheel_height = 1

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.3)

#create robot parts
body = patches.Rectangle((-body_width/2, -body_height/2), body_width, body_height, edgecolor='black', facecolor='blue')
left_wheel = patches.Rectangle((-wheel_radius, -wheel_height/2), wheel_radius*2, wheel_height, edgecolor='black', facecolor='orange')
right_wheel = patches.Rectangle((-wheel_radius, -wheel_height/2), wheel_radius*2, wheel_height, edgecolor='black', facecolor='green')

ax.add_patch(body)
ax.add_patch(left_wheel)
ax.add_patch(right_wheel)

ax.set_xlim(-50, 50)
ax.set_ylim(-50, 50)
ax.set_aspect('equal')
ax.grid(True)
ax.autoscale(False)

#initial state
pose = RobotPose()
dt = 0.01  # time step

#forward kinematic equations
def x_position(wl, wr, wheel_radius, theta):
    return ((wheel_radius * wl) / 2 * np.cos(theta) + (wheel_radius * wr) / 2 * np.cos(theta))

def y_position(wl, wr, wheel_radius, theta):
    return ((wheel_radius * wl) / 2 * np.sin(theta) + (wheel_radius * wr) / 2 * np.sin(theta))

goal_pose = [0.0, 0.0]
goal_marker = ax.plot(goal_pose[0], goal_pose[1], 'ro', markersize=6)[0]

def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        goal_pose[0] = event.xdata
        goal_pose[1] = event.ydata
        goal_marker.set_data([goal_pose[0]], [goal_pose[1]])

fig.canvas.mpl_connect('button_press_event', onclick)

#for animation
def animate(frame):
    to_goal_dx = goal_pose[0] - pose.x
    to_goal_dy = goal_pose[1] - pose.y
    euclidean_distance = np.sqrt(to_goal_dx**2 + to_goal_dy**2)
    angle_to_goal = np.arctan2(to_goal_dy, to_goal_dx)
    angle_difference = (angle_to_goal - pose.theta + np.pi) % (2 * np.pi) - np.pi  # normalize angle to [-pi, pi]

    #proportional control 
    v = kPlinear * euclidean_distance
    w = kPangular * angle_difference

    #change the float here for distance tolerance
    if euclidean_distance < 0.1:
        v = 0
        w = 0

    #inverse kinematics to compute wheel speeds
    wl = (2 * v - w * s) / (2 * wheel_radius)
    wr = (2 * v + w * s) / (2 * wheel_radius)

    #Forward kinematics to find actual robot velocities from wheel speeds
    v_actual = wheel_radius * (wr + wl) / 2
    w_actual = wheel_radius * (wr - wl) / s

    #update robot pose
    pose.x += v_actual * np.cos(pose.theta) * dt
    pose.y += v_actual * np.sin(pose.theta) * dt
    pose.theta += w_actual * dt

    #keep theta within [-pi, pi]
    pose.theta = (pose.theta + np.pi) % (2 * np.pi) - np.pi

    #add boundary kom oy robot jenh pi graph
    pose.x = np.clip(pose.x, -50, 50)
    pose.y = np.clip(pose.y, -50, 50)

    #calculate wheel positions in world frame
    left_wheel_local = np.array([0, s/2])
    right_wheel_local = np.array([0, -s/2])

    R = rotation_matrix_2d(np.degrees(pose.theta))
    robot_center = np.array([pose.x, pose.y])
    left_wheel_world = R @ left_wheel_local + robot_center
    right_wheel_world = R @ right_wheel_local + robot_center

    #update robot parts positions
    body.set_xy((pose.x - body_width / 2, pose.y - body_height / 2))
    left_wheel.set_xy((left_wheel_world[0] - wheel_radius, left_wheel_world[1] - wheel_height / 2))
    right_wheel.set_xy((right_wheel_world[0] - wheel_radius, right_wheel_world[1] - wheel_height / 2))

    t_b = mpl.transforms.Affine2D().rotate_around(pose.x, pose.y, pose.theta) + ax.transData
    t_lw = mpl.transforms.Affine2D().rotate_around(left_wheel_world[0], left_wheel_world[1], pose.theta) + ax.transData
    t_rw = mpl.transforms.Affine2D().rotate_around(right_wheel_world[0], right_wheel_world[1], pose.theta) + ax.transData

    body.set_transform(t_b)
    left_wheel.set_transform(t_lw)
    right_wheel.set_transform(t_rw)


#start animation
ani = FuncAnimation(fig, animate, interval=10)
plt.show()
