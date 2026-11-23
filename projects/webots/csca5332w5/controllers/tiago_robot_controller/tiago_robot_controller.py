"""
## DISCLAIMER

The simulation is sometimes unstable on my laptop with Webots R2025a.
The robot may break apart without an obvious cause. I have also noticed
different behavior after a fresh reboot compared with longer computer uptime.

Webots often prints this warning:
WARNING: The current physics step could not be computed correctly. Your world
may be too complex. If this problem persists, try simplifying your bounding
object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.

I cannot use a smaller basicTimeStep because the simulation becomes too slow.
I have not figured out why it is so slow; CPU resources seem sufficient.

If the robot breaks apart, please reset the simulation and retry at least once.

For the configuration space, I needed different convolution kernel sizes
at different basicTimeStep settings to meet the requirement for a
"one-pixel wide path (in narrow areas)". If your resulting map differs,
then the kernel size may need further adjustment.

A PNG showing the result on my computer is available here:
https://github.com/NKolotey/CUBoulder/blob/main/projects/webots/csca5332w5.png

Suggestions for improving simulation stability are welcome.
"""

from controller import Supervisor
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# actuators
motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# speed limits (likely equal)
MAX_SPEED_L = motor_left.getMaxVelocity()
MAX_SPEED_R = motor_right.getMaxVelocity()

# sensors
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

lidar_angles = np.linspace(
    lidar.getFov()/2, -lidar.getFov()/2,
    lidar.getHorizontalResolution())

# display
display = robot.getDevice('display')

# get the reference to the orange location
marker_node = robot.getFromDef("marker")
marker = marker_node.getField("translation")

# make the orange invisible to the lidar
lidar_node = robot.getFromDef("lidar")
marker_node.setVisibility(lidar_node, False)

# waypoints
WP = [
    [0.2, 0.2], [0.6, 0.0], [0.6,-1.5], [0.6, -3.0], [-0.6, -3.4],
    [-1.7,-3.0], [-1.7,-1.5], [-1.7, 0.1], [-0.6, 0.4]]
index = 0
index_dir = 1

# probabilistic world map
world_map = np.zeros((200, 300))
 
def world2map(xw, yw):
    p = np.asarray([xw + 2.35, 1.9 - yw]) * 50
    p = np.clip(p, 0, [199, 299]).astype(int)
    return p


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    # x, y of the slot (in world coordingates)
    xws, yws = gps.getValues()[:2]
    theta = np.arctan2(*compass.getValues()[:2])
    
    # x, y of the robot (in world coordingates)
    xwr = xws - 0.203*np.cos(theta)
    ywr = yws - 0.203*np.sin(theta)
    
    # trajectory
    px, py = world2map(xwr, ywr)
    display.setColor(0xFF0000)
    display.drawPixel(px, py)

    # get lidar range image and drop the first and last 80 readings
    lidar_ranges = np.array(lidar.getRangeImage())
    ranges = lidar_ranges[80:-80]
    angles = lidar_angles[80:-80]
    ranges[ranges == np.inf] = 100
    
    # translation matrix
    w_T_r = np.array([
        [np.cos(theta), -np.sin(theta), xws],
        [np.sin(theta), np.cos(theta),  yws],
        [0, 0, 1]
    ])

    X_i = np.array([
        ranges * np.cos(angles), 
        ranges * np.sin(angles), 
        np.ones(len(angles))])
    
    D = w_T_r @ X_i
    
    # update probabilities and visualize
    X, Y = D[0,:], D[1,:]
    for i in range(D.shape[1]):
        px, py = world2map(X[i], Y[i])
        world_map[px, py] += 0.01    
        
        v = int(world_map[px, py] * 255)
        v = v if v <= 255 else 255
        color = v*256**2 + v*256 + v
        
        display.setColor(color)
        display.drawPixel(px, py)
    
    # rho is the distance to the next waypoint
    # alpha is the remainin angle to turn
    rho = np.sqrt((xwr - WP[index][0])**2 + (ywr - WP[index][1])**2)
    alpha = np.arctan2(WP[index][1] - ywr, WP[index][0] - xwr) - theta

    # ensure alpha in in [-pi..pi] range
    if alpha > np.pi:
        alpha -= 2*np.pi
        
    if alpha < -np.pi:
        alpha += 2*np.pi

    # if the waypoint already very close
    if rho < 0.3:
        index += index_dir
        if index >= len(WP):
            index_dir = -1
            index = len(WP) - 1
            
        if index < 0:
            break
            
        else:
            marker.setSFVec3f([*WP[index],0.05])
            marker_node.resetPhysics()
    
    # a simple P-controller
    p1, p2 = 6, 4
    leftwheel = -p1*alpha + p2*rho
    rightwheel = p1*alpha + p2*rho
        
    leftwheel = np.clip(leftwheel, -MAX_SPEED_L, MAX_SPEED_L)
    rightwheel = np.clip(rightwheel, -MAX_SPEED_L, MAX_SPEED_R)
    
    motor_left.setVelocity(leftwheel)
    motor_right.setVelocity(rightwheel)


# out of the main loop, stop moving 
motor_left.setVelocity(0)
motor_right.setVelocity(0)

# this kernel size depends on the world/environment settings
kernel = np.ones((33, 33))
cmap = signal.convolve2d(world_map, kernel, mode='same')

cspace = cmap > 0.9
plt.imshow(cspace)
plt.show()
