"""maze_solver controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

def run_robot(robot):
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    time_step = 32
    max_speed = 6.28
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getMotor('motorname')
    #  ds = robot.getDistanceSensor('dsname')
    #  ds.enable(timestep)
    
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    left_ir = robot.getDistanceSensor('ir1')
    left_ir.enable(time_step)
    right_ir = robot.getDistanceSensor('ir2')
    right_ir.enable(time_step)
    front_ir = robot.getDistanceSensor('ir0')
    front_ir.enable(time_step)
    # right_front_ir = robot.getDistanceSensor('ir0')
    # right_front_ir.enable(time_step)
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()
        front_ir_value = front_ir.getValue()
 
        # Enter here functions to read sensor data, like:
        print("left:{} right:{} front:{}".format(left_ir_value,right_ir_value,front_ir_value))
        
        left_speed = max_speed *.5
        right_speed = max_speed *.5
        print("{}".format(left_speed))
        left_move = left_ir_value > 700
        right_move = right_ir_value > 700
        front_move = front_ir_value > 700
        #  val = ds.getValue()
        if left_move:
            left_speed = -max_speed * 0.2
            right_speed = max_speed *.5
        elif front_move:
            left_speed = max_speed *.5
            right_speed = max_speed *.5
        elif right_move:
            left_speed = max_speed *.5
            right_speed = -max_speed * 0.2
        else:
            left_speed = max_speed 
            right_speed = -max_speed
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        # pass

# Enter here exit cleanup code.
if __name__ == "__main__":
    robot =Robot()
    run_robot(robot)
