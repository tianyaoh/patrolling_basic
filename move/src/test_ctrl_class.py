#!usrbinenv python

import rospy
from geometry_msgs.msg import Twist
import time

# rospy.init_node('move_forward_publisher')


# frw_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# starting_time = rospy.Time.now()

# while not rospy.is_shutdown()
#     twist = Twist()
#     twist.linear.x = 0.2
#     frw_pub.publish(twist)


class BotControl()

    def __init__(self,robot_name = turtlebot)
        # initializing python as node
        rospy.init_node('control_node')

        # print to console 
        rospy.loginfo(This is turtlebot at your command)
        
        # topic,msg_type,publisher, &subscriber
        self.cmd_vel_topic = cmd_vel
        self.cmd = Twist()

        self.front_scan_topic = 'frontscan'
        self.back_scan_topic = 'backscan'
        self.right_scan_topic = 'rightscan'
        self.left_scan_topic = 'leftscan'

        # start publisher
        self.cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # coordinate disposition
        self.disposition = {disp_lx0,disp_ly0,disp_lz0,disp_ax0,disp_ay0,disp_az0}

        # This is the state of motion, boolean
        # now in_motion, not_in_motion
        # future moving,airborne,sliping,sinking,rolling
        self.moving = False
        # self_stop
        self.rate = rospy.Rate(1)

    def move_straight_for_x(self,speed,direction,time)
        # ensuring positvity for speed
        speed = abs(speed)
        # rospy.loginfo(initial speed %s,speed)
        if direction == 'backward'
            speed = -speed
        
        i = 0
        self.moving = True


        # loop to publish the velocity estimate, current_distance = velocity  (t1 - t0)
        while (i = time)
            # Publish the velocity
            self.motion(lx = speed)
            i += 1
            self.rate.sleep()

        self.stop()


    def move_straight(self,speed,direction='forward')
        # ensuring positvity for speed
        speed = abs(speed)
        # rospy.loginfo(initial speed %s,speed)
        if direction == 'backward'
            speed = -speed
        # publishing velocity
        self.motion(lx = speed)
        self.moving = True

    def turn_time(self,direction,speed,time)
        speed = abs(speed)
        if direction == 'counter_clockwise'
            speed = -speed
        
        i = 0
        self.moving = True
        # loop to publish the velocity estimate, current_distance = velocity  (t1 - t0)
        while (i = time)
            # Publish the velocity
            self.motion(az = speed)
            i += 1
            self.rate.sleep()
        self.stop()

    def turn(self,direction,speed)
        speed = abs(speed)
        if direction == 'counter_clockwise'
            speed = -speed
        self.motion(az = speed)

    def publish_once_in_cmd_vel(self)
        # This function is taken from the Python-Robotic class from Construct.com
        #   From the 4th chapter about function, under the class bot_control, function publish_once in cmd_vel
        #   There is little change for variable names but structure is kept as original.
        while not rospy.is_shutdown()
            connections = self.cmd_publisher.get_num_connections()
            if connections  0
                self.cmd_publisher.publish(self.cmd)
                #rospy.loginfo(Cmd Published)
                break
            else
                self.rate.sleep()

    # privat function should only be used internally
    def motion(self,lx = 0,ly =0,lz = 0,ax = 0,ay = 0,az = 0)
        # updateing cmd
        self.cmd.linear.x = lx
        self.cmd.linear.y = ly
        self.cmd.linear.z = lz
        self.cmd.angular.x = ax
        self.cmd.angular.y = ay
        self.cmd.angular.z = az
        # rospy.loginfo(lx = %s ly = %s lz =  %s ax = %s ay = %s az =  %s ,lx,ly,lz,ax,ay,az)
         # publishing in ignorance of connection
        self.publish_once_in_cmd_vel()
        self.moving = True

    def move_towards_x_y(x,y)
        pass
        # write action client

    # def distance_in_theory(x_axis,speed,t1,t2)
    #     temp = speedabs(abs(t1)-abs(t2))
    #     self.disposition[x_axis] += temp
    #     return temp

    def stop(self)
        self.motion()
        self.moving = False
        # rospy.loginfo(Bot internally not in motion)
