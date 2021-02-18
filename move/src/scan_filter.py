#!/usr/bin/env python
import rospy

# import control from test_ctrl_class
from sensor_msgs.msg import LaserScan;

RANGE_MINIMUM = 0.119999997318
RANGE_MAXIMUM = 3.5

MAX_PLACEHOLDER = 100.0
MIN_PLACEHOLDER = 50.0


# range_filter takes in RAW data, returns filtered scan data based on constant scanner min and max
#   input: list(LaserScan.ranges),type: list;
#   output: ranges, type:list;
def range_filter(ranges):
    # loop through data with index
    for x in range(len(ranges)):
        # rospy.loginfo("type of RANGE: %s",type(RANGE_MAXIMUM))
        # rospy.loginfo("type of ranges:%s",type(ranges))
        if ranges[x] <= RANGE_MINIMUM:
            ranges[x] = MIN_PLACEHOLDER
        elif ranges[x] >=MAX_PLACEHOLDER:
            ranges[x] = MAX_PLACEHOLDER
    return ranges

# LaserScan_msg_create takes in filtered range segments, created segmented scan messages, and return it
#   input: filtered_range,type: list;
#   output:  temp_msg,type: Object-LaserScan();
def LaserScan_msg_create(filtered_range):
    temp_msg = LaserScan()

    # constant angle increment
    temp_msg.angle_increment = 0.0175019223243 #approximately 1 degree change
    # side range data
    temp_msg.ranges = filtered_range

    # uncomment to test side range length and its content
    # rospy.loginfo("length of new ranges(90): %s", len(temp_msg.ranges))
    # rospy.loginfo(temp_msg.ranges)
    
    # calculating min and max
    temp_msg.range_min = range_min(temp_msg.ranges)
    temp_msg.range_max = range_max(temp_msg.ranges)
    return temp_msg

# range_min calculate min in filtered scan data, returns it
#   input: range_segments,type: list;
#   output:  temp_min, type: int;
def range_min(range_segments):
    # min_placeholder is definitely maximum in filtered scan data - range_segments
    temp_min = MIN_PLACEHOLDER
    # looping through floats
    for num in range_segments:
        # ignoring placeholders
        if num > (MIN_PLACEHOLDER-1):
            pass
        else:
            # if number is less than current min, replace it
            if num < temp_min:
                temp_min = num
    return temp_min 

# range_max calculate max in filtered scan data, returns it
#   input: range_segments,type: list;
#   output:  temp_max, type: int;
def range_max(range_segments):
    # 0 is absolute min in filtered data
    temp_max = 0
    # looping through floats
    for num in range_segments:
        if num > (MIN_PLACEHOLDER-1):
            pass
        else:
            # if number is greater than current max, replace it
            if num > temp_max:
                temp_max = num
    return temp_max

# Testing segmented msgs, focusing on .ranges, .range_min, .range_max
#   input: test_msg,type: Object - LaserScan();
#   output:  None
def msg_test(test_msg):
    # rospy.loginfo("msg ranges: %s",test_msg.ranges)
    rospy.loginfo("msg max: %s",test_msg.range_max)
    rospy.loginfo("msg min: %s",test_msg.range_min)
    # rospy.loginfo("msg test complete")

# CALLBACK - sub to scan data, filtering RAW scan data, 
#               publishing filtered segmented scan data to different side topics
def callback(msg):
    rospy.loginfo("call back starts here")
    # rospy.loginfo("type of msg_ranges:%s",type(msg.ranges))

    # initial filtering
    temp_list = range_filter(list(msg.ranges))
    # rospy.loginfo("FULL temp %s", temp_list)
    
    # front 335-25
    frt_msg = LaserScan_msg_create([x for segment in [temp_list[335:],temp_list[:25]] for x in segment])
    # frt_msg = LaserScan_msg_create([x for segment in [temp_list[315:],temp_list[:45]] for x in segment])
    # rospy.loginfo("publishing front_side scan")
    scan_frt_pub.publish(frt_msg)
    
    # uncomment next line to test msgs
    msg_test(frt_msg)

    # right 225-335

    rgt_msg = LaserScan_msg_create(temp_list[225:335])
    # rospy.loginfo("publishing right_side scan")
    scan_rgt_pub.publish(rgt_msg)

    # uncomment next line to test msgs
    # msg_test(rgt_msg)

    # back 135-225
    bck_msg = LaserScan_msg_create(temp_list[135:225])
    # rospy.loginfo("publishing back_side scan")
    scan_bck_pub.publish(bck_msg)

    # uncomment next line to test msgs
    # msg_test(bck_msg)

    # left 25-135 
    lft_msg = LaserScan_msg_create(temp_list[25:135])
    # rospy.loginfo("publishing left_side scan")
    scan_lef_pub.publish(lft_msg)

    # uncomment next line to test msgs
    # msg_test(lft_msg)

    # TEST is the ranges are segmented correctly
    # temp = [x for segment in [temp_list[315:],temp_list[:45]] for x in segment]
    # rospy.loginfo("type of temp %s",type(temp))
    # rospy.loginfo("length of temp %s",len(temp))
    # rospy.loginfo("type of frt_msg.ranges %s",type(frt_msg.ranges))
    # rospy.loginfo("length of frt_msg.ranges %s",len(frt_msg.ranges))
    # rospy.loginfo(frt_msg.ranges)
    # rospy.loginfo("TESTING range contents: %s", temp ==frt_msg.ranges)

# init node 
rospy.init_node('scan_filter')

# sub to RAW scan data
scan_sub = rospy.Subscriber('scan',LaserScan,callback)

# there could be a customized message sent with segmented front, left_side,right_side, rear side segment. Maybe not neccessary
scan_frt_pub = rospy.Publisher('/front/scan', LaserScan, queue_size = 3)
scan_bck_pub = rospy.Publisher('/back/scan', LaserScan, queue_size = 3)
scan_rgt_pub = rospy.Publisher('/right/scan', LaserScan, queue_size = 3)
scan_lef_pub = rospy.Publisher('/left/scan', LaserScan, queue_size = 3)

rospy.spin()