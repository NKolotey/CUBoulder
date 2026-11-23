"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# implementation constants
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.02
D = 0.052

# get the instances of the motors
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# and the sensors
gs = []
for i in range(3):
    sensor = robot.getDevice(f'gs{i}')
    sensor.enable(timestep)
    gs.append(sensor)


xw, yw, alpha = 0, 0, 0
phildot, phirdot = 0, 0
stopped = False
counter = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if stopped:
        continue

    # Read the sensors:
    g = [sensor.getValue() for sensor in gs]
    
    # If over the line and after (at least almost) full clockwise rotation
    if g[0] < 350 and g[1] < 350 and g[2] < 350 and alpha < -0.9*2*math.pi:
        counter = counter + 1
        
        # If three consequitive readings
        if counter >= 3:
            stopped = True
            phildot, phirdot = 0, 0
            
            print('Over the line!')
            print(np.sqrt(xw**2+yw**2))            
    else:
        counter = 0
    
    # Drive forward
    if g[0] > 500 and g[1] < 350 and g[2] > 500:
        phildot, phirdot = MAX_SPEED, MAX_SPEED
        
    # Turn right
    elif g[2] < 550 and g[0] > 550:
        phildot, phirdot = 0.25 * MAX_SPEED, -0.1 * MAX_SPEED
        
    # Turn left
    elif g[0] < 550 and g[2] > 550:
        phirdot, phildot = 0.25 * MAX_SPEED, -0.1 * MAX_SPEED

    motor_left.setVelocity(phildot)
    motor_right.setVelocity(phirdot)
    
    deltaT = timestep/1000    
    deltaX = (phildot + phirdot) / 2 * WHEEL_RADIUS * deltaT
    deltaomegaz = (phirdot - phildot) / D * WHEEL_RADIUS * deltaT
    
    xw = xw + np.cos(alpha)*deltaX
    yw = yw + np.sin(alpha)*deltaX
    alpha = alpha + deltaomegaz
    
    if not stopped:
        print(f"xw={xw}, yw={yw}, alpha={alpha}")
