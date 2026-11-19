"""state_machine_based_controller controller."""

from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# implementation constants
MAX_SPEED = 6.28
DRIVE_SPEED = MAX_SPEED * 0.5
ROTATE_SPEED = MAX_SPEED * 0.05
ROTATE_180_DIFF = 4.4588
DIST_5CM = 80


# get the instances of the motors
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

# and the sensors
wheel_sensor_left = robot.getDevice('left wheel sensor')
wheel_sensor_right = robot.getDevice('right wheel sensor')
wheel_sensor_left.enable(timestep)
wheel_sensor_right.enable(timestep)

distance_sensors = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(timestep)
    distance_sensors.append(sensor)


state = "FORWARD_1"
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
motor_left.setVelocity(DRIVE_SPEED)
motor_right.setVelocity(DRIVE_SPEED)

# number of consecutive "no wall at left" in FOLLOW_WALL state
counter = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    dist = [sensor.getValue() for sensor in distance_sensors]
    wheel_left = wheel_sensor_left.getValue()
    wheel_right = wheel_sensor_right.getValue()
    
    if state == "FORWARD_1":
        if (dist[0] + dist[7]) / 2 > DIST_5CM:
            state = "U_TURN"
            motor_right.setVelocity(ROTATE_SPEED)
            motor_left.setVelocity(-ROTATE_SPEED)
            target_wheel_right = wheel_right + ROTATE_180_DIFF
            target_wheel_left = wheel_left - ROTATE_180_DIFF

    elif state == "U_TURN":
        if wheel_left < target_wheel_left or wheel_right > target_wheel_right:
            state = "FORWARD_2"
            motor_right.setVelocity(DRIVE_SPEED)
            motor_left.setVelocity(DRIVE_SPEED)

    elif state == "FORWARD_2":
        if (dist[0] + dist[7]) / 2 > DIST_5CM:
            state = "TURN_RIGHT"
            motor_right.setVelocity(-ROTATE_SPEED)
            motor_left.setVelocity(ROTATE_SPEED)
                
    elif state == "TURN_RIGHT":
        if dist[5] > DIST_5CM:
            state = "FOLLOW_WALL"
            motor_right.setVelocity(DRIVE_SPEED)
            motor_left.setVelocity(DRIVE_SPEED)
            
    elif state == "FOLLOW_WALL":
        d7 = max(0, dist[7] - DIST_5CM)
        d6 = max(0, dist[6] - DIST_5CM)
        davg = d7 * 0.7 + d6 * 0.3        
        m = 0.9 + 0.05 * davg
        
        motor_right.setVelocity(DRIVE_SPEED)
        motor_left.setVelocity(DRIVE_SPEED * m)
        
        counter = counter + 1 if dist[5] < DIST_5CM else 0
        if counter >= 5:
            state = "STOP"
            motor_right.setVelocity(0)
            motor_left.setVelocity(0)

    elif state == "STOP":
        pass
