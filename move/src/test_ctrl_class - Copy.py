#!/usr/bin/env python
import rospy;
import math;
import time;
import numpy as np;
from test_ctrl_class import BotControl;
from sensor_msgs.msg import LaserScan;
from nav_msgs.msg import Odometry;
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# surrounding_status: 
#     front_status
#     left_status
#     right_status
#     back_status

LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.4
PI = 3.1415926535857932
NINETY_DEGREE_RAD = PI/2

# Determine whether robot needs to turn right or left
#   input: right_min,type: int left_min,type: int;
#   output:  str counter_clockwise or clockwise
def left_or_right(right_min,left_min):
    str = 'clockwise'
    if left_min <= right_min:
        str = 'counter_clockwise'             
    return str

# ***RETURN_FUNCTION*** rotation_direction, given curr_angle, rad_angle caclulate whether to turn left or right
#   input: cuur_angle,type: float; rad_angle,type: float;
#   output:  string, counter_clockwise, type: String;
def rotate_direction(curr_angle, rad_angle):
    # half circle algorithem, with two pointers, rad_angle 
    
    # calculate target angle complement:
    if rad_angle >= 0:
        rad_angle_complement = -(PI-rad_angle)
    else:
        rad_angle_complement = PI+rad_angle
    print("angle_complement",rad_angle_complement)

    # if rad_angle is positive, probe (counter_clockwise) positive, and from -pi probe negative with complement
    if rad_angle >= 0:
        # within range go right
        if curr_angle > rad_angle or curr_angle < rad_angle_complement:
            # print("positive on the right")
            return "counter_clockwise"
        else:
            # print("positive on the left")
            return "clockwise"
    # if rad_angle is negative, prob clockwise. from target angle to -pi, from pi to complement
    else:
        if curr_angle < rad_angle or curr_angle > rad_angle_complement:
            # print("negative go right")
            return "clockwise"
        else:
            # print("negative go left")
            return "counter_clockwise"

# ***RETURN_FUNCTION*** calculate euclidean distance with pose, sqrt(x**2,y**2)
#   input: pose, type: float;
#   output:  straight_line distance from current position
def euclidean_distance(pose):
    return math.sqrt(pow((pose.position.x), 2) + pow((pose.position.y), 2))

# ***RETURN_FUNCTION*** angle_calc, placing pointer to converted euclid angle circle with positive from positive x, crossing positive y, to negative x (x+, 0; to; x-)
#   input: x, type: float; y, type: float
#   output:  euclide pointer type: float
def angle_calc(x,y):
    if (x > 0) and (y > 0): 
        print("go 4")
        return -NINETY_DEGREE_RAD-math.atan(x/y)
    elif (x<0) and (y<0):
        print("go 1")
        return NINETY_DEGREE_RAD-math.atan(x/y)        
    elif (y<0) and (x>0):
        print("go 3")
        return NINETY_DEGREE_RAD+abs(math.atan(x/y))        
    elif (y>0) and (x<0):
        print("go 2")
        return -NINETY_DEGREE_RAD+abs(math.atan(x/y))
    elif (x==0) and (y > 0):
        return -NINETY_DEGREE_RAD
    elif (x==0) and (y <0):
        return NINETY_DEGREE_RAD
    elif (y==0) and (x > 0):
        return PI
    elif (y==0) and (x <0):
        return 0
    else:
        print("this should never be printed, look at angle_calc")

# ***RETURN_FUNCTION*** return if a is close to b for abs_tolerance
#   input: x, type: float; y, type: float
#   output: Boolean: True or False
def isClose(a,b,abs_tol):
    # If a,b has the same sign
    if (a > 0 and b >0 ) or (a <= 0 and b <= 0):
        if a >= b:
            return True if a-b <= abs_tol else False
        else:
            return True if b-a <= abs_tol else False 
    elif a == 0 or b == 0:
        return True if abs(a-b) <= abs_tol else False
    else:
        # if the sign are close, they are only approximately close if their combined distance to zero is less than abs tolerance
        print(True if (abs(a)+abs(b))<abs_tol else False)
        return True if (abs(a)+abs(b))<abs_tol else False

  

# ***RETURN_FUNCTION_MAIN*** running place of returning to zero pose
#   input: curr_odom, type: Odometry; botcontrol, type: BotControl
#   output: Nothing
def return_to_zero(curr_odom,botcontrol):

    # initializing pointer angle, and distance bewteen
    flagged_angle = angle_calc(curr_odom.pose.pose.position.x,curr_odom.pose.pose.position.y)
    distance = euclidean_distance(curr_odom.pose.pose)


    # print("initial distance",distance)
    # print("distance check",isClose(0,distance,abs_tol = 0.25))

    # loop stops if distance is close enough to orgin
    while not(isClose(0,distance,abs_tol = 0.25)):
        curr_msg = rospy.wait_for_message("/odom", Odometry, timeout=None)
        curr_pose = curr_msg.pose.pose
        curr_angle = (rospy.wait_for_message("/yaw_euler/odom",Float32, timeout=None)).data
        flagged_angle = angle_calc(curr_odom.pose.pose.position.x,curr_odom.pose.pose.position.y)
        distance = euclidean_distance(curr_pose)
        print('flagged angle',flagged_angle)
        print("curr_angle",curr_angle)

        # stops turning and go straight if angle is close to flag pointer
        if isClose(flagged_angle, curr_angle,abs_tol=0.1):
            print("distance updates: ",distance)
            print("walking")
            botcontrol.move_straight_for_x(0.1,'forward',1)
        else:
            # turn for 1 second
            print("not facing the right way")
            botcontrol.turn_time(rotate_direction(curr_angle,flagged_angle),0.1,1)


    botcontrol.stop()
    print("DONEEEEE")

    




def patrol(patroll_time,botcontrol):
    # bot confused when both side are equally close, off setting angle
    botcontrol.turn_time('clockwise',ANGULAR_SPEED,2)
    while ((time.time() - start_time) < patroll_time.to_sec()) and (not rospy.is_shutdown()):
    # while not rospy.is_shutdown():
        # Grabing messsages from scan
        frt_msg = rospy.wait_for_message("/front/scan", LaserScan, timeout=5)
        rgt_msg = rospy.wait_for_message("/right/scan", LaserScan, timeout=5)
        lft_msg = rospy.wait_for_message("/left/scan", LaserScan, timeout=5)

        # handling "too-close" close situation, if front scan max is great enough,or if front range is too close
        if frt_msg.range_max > 0.4 or frt_msg.range_min > 0.20:
            # if there are some distance between front obstacle and robot,
            #  if robot is close but 0.3 is a turnable distance
            if frt_msg.range_min < 0.4:
                print("stop and turn")
                # bot confused when both side are equally close
                # if abs(lft_msg.range_min-rgt_msg.range_min) < 0.1:
                #     botcontrol.turn_time('clockwise', ANGULAR_SPEED,5)
                # else:
                    # turn left or right
                botcontrol.turn(left_or_right(rgt_msg.range_min,lft_msg.range_min), ANGULAR_SPEED)
                if frt_msg.range_min > 0.40:
                    botcontrol.move_straight(LINEAR_SPEED)
                    print("moving straight again")

            # we are still not close enough(0.3), keep moving forward
            else:
                botcontrol.move_straight(LINEAR_SPEED)
                print("keep moving straight")

        # backing off when we are too close
        else:
            print("stuck and trying to fall back")
            botcontrol.move_straight_for_x(LINEAR_SPEED/2,"backward",2)
            botcontrol.turn_time(left_or_right(rgt_msg.range_min,lft_msg.range_min), ANGULAR_SPEED,3)

        # print("end",(time.time() - start_time))
        time.sleep(1)




if __name__ == '__main__': 
    botcontrol = BotControl()
    patroll_time = rospy.Duration.from_sec(60)
    start_time = time.time()

    patrol(patroll_time,botcontrol)
    print("patrol done")
    botcontrol.stop()
    curr_odom = rospy.wait_for_message("/odom", Odometry, timeout=None)

    return_to_zero(curr_odom,botcontrol)

    if botcontrol.moving == True:
        botcontrol.stop()

# side of Clear
# side of Dirty

# Radar Solution
# while not rospy.is_shutdown():

    # problem: this could be bad if the first found boundary is a column (then the bot would always circulating the cylinder)

    # Possible Optimization: find closest wall/edge; Challenge: How to find an edge from lidar data(round depth sensor)
    # first find a boudary to stick to.

    # stick near the wall/ an edge until the next turn (assuming enclosed space)
    #   during this state, robot should in a relatively straight route (assuming followed axis is X)
    #   during this process, robot drops pins along its path ; 
    #               ** try employ double queue
    #          robot sensor error: sensor finds space, but robot cannot path over obstacle
                    # write some code for this senario 
        
        # whlie:
            # go along the path if there is a corresponding, dequeued pin from y-dispostion or (clear-side is closesly "followed in relative distance"), path is not blocked, or 

        # new wall/obstacle reached, terminate route pining, follow new edge(keep object on right/left) (obstacle could be big- wall or small chair leg) 
        #           if y_displacement is > ??? while going around, turn back go in new path, else if it is less than ???, keep following until y falls back into original y track

# shortest returning algorithem
