#Hello this is a test, this is a test

#jenkins should read this////

To run program, unzip all files and make sure roscore is running and gazebo is launched properly!

run 2 nodes first: rosrun move scan_filter.py; rosrun quaternion_to_euler.py

nothing should print in quaternion's window, 
but in scan_filter terminal, you should be able to see:

e.g
call back starts here:
[INFO] [196541115651, 646548918435481] msg max: 2.45
[INFO] [196541115651, 646548918435481] msg min: 0.73
call back starts here:
[INFO] [196541115651, 646548918435481] msg max: 2.45
[INFO] [196541115651, 646548918435481] msg min: 0.73
call back starts here:
[INFO] [196541115651, 646548918435481] msg max: 2.45
[INFO] [196541115651, 646548918435481] msg min: 0.73

 
next, run main program: rosrun move test_procedure.py
if connected properly you should be able to see the following:

e.g:

This is turtlebot at your command
keep moving straight
keep moving straight
keep moving straight
keep moving straight
stuck and trying to fall back
stop and turn
keep moving straight
keep moving straight
keep moving straight
keep moving straight
keep moving straight
keep moving straight
patrol done
go 2
('flagged angle', -1.2046651)
('curr_angle',0.649818564)
False
not facing the right way
('angle_complement', 1.58418187741)
go 2
('flagged angle', -1.2046651)
('curr_angle',0.649818564)
False
not facing the right way
('angle_complement', 1.58418187741)
go 2
('flagged angle', -1.2046651)
('curr_angle',0.649818564)
('angle_complement', 1.58418187741)
walking
('flagged angle', -1.2046651)
('curr_angle',0.649818564)
('angle_complement', 1.58418187741)
('flagged angle', -1.2046651)
walking
DONEEEEE

---end_of_terminal_msg----

Explanation of algorithem:
patrol:
	#bot is initially turned, because a bug, if bot is perfectly straight into a wall, it keeps turn left and right

	determining if there is space at front, if so
		move forward
	if not:
		if left min is close
			turn right
		if right min is close
			turn left
	else if front space is too close, 
		fall back

return:
	while bot is not close to zero
		if angle is not close to target
			turn
		else 
			move forward

return:
	angle_calculation:
		determining the target angle in four different quadrents and how they are suppose to be calculated
		all is converted to euclid angle
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

# ***RETURN_FUNCTION*** rotation_direction, given curr_angle, 
			rad_angle caclulate 
			whether to turn left or right

This function uses 180 probing in one direction from target angle, if there curr_angle is within this range, turn left/righ, else turn the other way

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

