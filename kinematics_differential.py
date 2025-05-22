import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from matplotlib.widgets import Slider

fig, ax = plt.subplots()

def rotation_matrix_2d(theta_deg):
    theta = np.radians(theta_deg)
    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    return R

body_width = 3
body_height = 2
s = 3 #distance between the center of each wheels
wheel_radius = 0.5
wheel_height = 1

axbx = plt.axes([0.25, 0.2, 0.65, 0.03])
axby = plt.axes([0.25, 0.15, 0.65, 0.03])
axtheta = plt.axes([0.25, 0.1, 0.65, 0.03])

bx_slider = Slider(axbx,'x',-10,10,valinit=0)
by_slider = Slider(axby,'y',-10,10,valinit=0)
theta_slider = Slider(axtheta, 'theta', 0, 180, valinit=0, valstep=1)

body = patches.Rectangle((0-(body_width/2), 0 -(body_height/2)), body_width, body_height, edgecolor='black', facecolor='blue')
left_wheel = patches.Rectangle((0-(wheel_radius), 0-(wheel_height/2)), wheel_radius*2, wheel_height, edgecolor='black', facecolor='orange')
right_wheel = patches.Rectangle((0-(wheel_radius), 0-(wheel_height/2)), wheel_radius*2, wheel_height, edgecolor='black', facecolor='green')

body_center = np.array([0,0])

def update(val):
    bx = bx_slider.val
    by = by_slider.val
    theta = theta_slider.val
    robot_center = np.array([bx,by])
    left_wheel_local = np.array([0, s/2])
    right_wheel_local = np.array([0, -s/2])

    left_wheel_world = rotation_matrix_2d(theta) @ left_wheel_local + robot_center
    right_wheel_world = rotation_matrix_2d(theta) @ right_wheel_local + robot_center

    body.set_xy((robot_center[0] - body_width/2, robot_center[1] - body_height/2))
    left_wheel.set_xy((left_wheel_world[0] - wheel_radius, left_wheel_world[1] - wheel_height/2))
    right_wheel.set_xy((right_wheel_world[0] - wheel_radius, right_wheel_world[1] - wheel_height/2))

    t_b = mpl.transforms.Affine2D().rotate_deg_around(robot_center[0], robot_center[1], theta) + ax.transData
    t_l_w = mpl.transforms.Affine2D().rotate_deg_around(left_wheel_world[0], left_wheel_world[1], theta) + ax.transData
    t_r_w = mpl.transforms.Affine2D().rotate_deg_around(right_wheel_world[0], right_wheel_world[1], theta) + ax.transData
    body.set_transform(t_b)
    left_wheel.set_transform(t_l_w)
    right_wheel.set_transform(t_r_w)
    
    fig.canvas.draw_idle()

bx_slider.on_changed(update)
by_slider.on_changed(update)
theta_slider.on_changed(update)

ax.add_patch(body)
ax.add_patch(left_wheel)
ax.add_patch(right_wheel)

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect('equal')    
ax.grid(True)
ax.autoscale(False)       

bx_slider.on_changed(update)
by_slider.on_changed(update)
theta_slider.on_changed(update)

update(None)              
plt.show()