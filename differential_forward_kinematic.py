import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
import keyboard
import time
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass

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

#sliders
axw = plt.axes([0.25, 0.2, 0.65, 0.03])
w_slider = Slider(axw, 'w, max angular velocity', 0, 50, valinit=0)

#reset button
axreset = axwr = plt.axes([0.25, 0.1, 0.65, 0.03])
button = Button(axreset, 'Reset', hovercolor='0.975')

#create robot parts
body = patches.Rectangle((-body_width/2, -body_height/2), body_width, body_height, edgecolor='black', facecolor='blue')
left_wheel = patches.Rectangle((-wheel_radius, -wheel_height/2), wheel_radius*2, wheel_height, edgecolor='black', facecolor='orange')
right_wheel = patches.Rectangle((-wheel_radius, -wheel_height/2), wheel_radius*2, wheel_height, edgecolor='black', facecolor='green')

ax.add_patch(body)
ax.add_patch(left_wheel)
ax.add_patch(right_wheel)

ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
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

def reset(event):
    w_slider.reset()

#for animation
def animate(frame):
    if keyboard.is_pressed('left'):
        wl = -w_slider.val/2
        wr = w_slider.val/2
    elif keyboard.is_pressed('right'):
        wl = w_slider.val/2
        wr = -w_slider.val/2
    elif keyboard.is_pressed('up'):
        wl = wr = w_slider.val
    elif keyboard.is_pressed('down'):
        wl = wr = -w_slider.val
    else: 
        wl = wr = 0

    #external forward kinematics equation
    dx = x_position(wl, wr, wheel_radius, pose.theta) * dt
    dy = y_position(wl, wr, wheel_radius, pose.theta) * dt
    dtheta = (-(wheel_radius / s) * wl + (wheel_radius / s) * wr) * dt

    pose.x += dx
    pose.y += dy
    pose.theta += dtheta

    pose.x = np.clip(pose.x, -20, 20)
    pose.y = np.clip(pose.y, -20, 20)

    #local-to-world wheel positions
    left_wheel_local = np.array([0, s/2])
    right_wheel_local = np.array([0, -s/2])

    R = rotation_matrix_2d(np.degrees(pose.theta))
    robot_center = np.array([pose.x, pose.y])
    left_wheel_world = R @ left_wheel_local + robot_center
    right_wheel_world = R @ right_wheel_local + robot_center

    #update patch positions
    body.set_xy((pose.x - body_width / 2, pose.y - body_height / 2))
    left_wheel.set_xy((left_wheel_world[0] - wheel_radius, left_wheel_world[1] - wheel_height / 2))
    right_wheel.set_xy((right_wheel_world[0] - wheel_radius, right_wheel_world[1] - wheel_height / 2))

    #applying transform so the patch rotates correctly
    t_b = mpl.transforms.Affine2D().rotate_around(pose.x, pose.y, pose.theta) + ax.transData
    t_lw = mpl.transforms.Affine2D().rotate_around(left_wheel_world[0], left_wheel_world[1], pose.theta) + ax.transData
    t_rw = mpl.transforms.Affine2D().rotate_around(right_wheel_world[0], right_wheel_world[1], pose.theta) + ax.transData

    body.set_transform(t_b)
    left_wheel.set_transform(t_lw)
    right_wheel.set_transform(t_rw)

# start animation
ani = FuncAnimation(fig, animate, interval=10)
button.on_clicked(reset)
plt.show()
