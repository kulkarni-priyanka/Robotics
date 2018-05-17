#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps,radians
import math
import time
import asyncio
from Lab.lab6 import pose_transform
import numpy as np
# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	'''
	Method to identify radius: Used cozmo_drive_straight(robot, 172, 30) to observe the number of full rotations.
	After multiple runs, concluded that 172 mm = 2 full rotations
	Thus, 172 = 2*(circumference) where cirumference = 2* pi*r
	And thius computed radius r	
	'''

	radius = (172/2)/(2*math.pi)
	#print(radius)
	return radius

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""

	'''
	I set different speeds for left and wheels
	Running robot.drive_wheels(5,30), I observed that the time taken to complete a circle = 25.2 seconds
	We now have two circles traced (one inner by left wheel and one ouuter by right)
	since distance = speed * time, we have circumference_outer i.e 2*pi*r_outer = 30 * 25.2 and computed outer radius from this equation
	similarly, inner radius can be obtained. 
	The distance between wheels is the difference between the outer circle radius and the inner circle radius	
	'''

	radius_inner = (25.2*5)/(2*math.pi)
	radius_outer = (25.2*30)/(2*math.pi)
	distance_between_wheels = radius_outer-radius_inner
	return distance_between_wheels

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	'''
	We simply want to translate the angle in degrees to a distance to be covered, which is the arc length of the circle.
	The arc length is the measure of the distance along the curved line making up the arc. It is longer than the straight line distance between its endpoints (which would be a chord) 
	Arc_length = (2*pi*r)*((angle_deg/360))
	If we drive straight with distance = arc_length, we can verify that the front wheel roates by the expected degrees.	
	'''

	arc_length = (angle_deg/360)* (2*math.pi*get_front_wheel_radius())
	cozmo_drive_straight(robot,arc_length,5)

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""

	'''
	Given distance and speed, we can compute time = distance/speed
	This is the time for which we have to rotate Cozmo's wheels. 
	And since we are expected to drive straight, the same speed is applied to both left and right wheels.	
	'''
	time = dist/speed
	robot.drive_wheels(speed,speed,duration=time)

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""

	'''
	In order to turn in place with a given angle, we see that the distance covered in mm is equal to arc_length of the circle with radius
	equal to the size of robot (distance between its wheels).
	Outer distance/Arc_length = (2*pi*r)*((angle_deg/360)) 
	
	Now, the function takes speed input in degrees per second and drive_wheels needs speed in mm/s
	it takes (360/(given speed)) seconds to complete full circle (2*pi*r)
	Hence to cover 1mm, we need new speed as:  (speed/360)*(2*math.pi*(radius))
	Where radius is distance between wheels.
	
	With the arc_lenth and speed in mm we can now compute time as distance/speed
	
	If the angle is positive we need to rotate left and thus we set left wheel speed to 0 and right wheel speed to speed_mm.
	And do the converse when the angle is negataive
	'''

	distance_outer = (angle/360)* (2*math.pi*(get_distance_between_wheels()))
	speed_mm = (speed/360)*(2*math.pi*(get_distance_between_wheels()))
	time = abs(distance_outer / speed_mm);

	if(angle > 0):
		robot.drive_wheels(0,speed_mm,duration=time)
	else:
		robot.drive_wheels(speed_mm,0, duration=time)


def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the my_drive_straight and my_turn_in_place functions. This should
	# include a sequence of turning in place, moving straight, and then turning
	# again at the target to get to the desired rotation (Approach 1).
	# ####

	'''
	
	Initial steps: Compute the distance between current and target position.
	We can visualize this as trying to fund the hypotenuse when the c
	
	Step1: Turning to face the target co-ordinates
		We can visualize this motion as trying to find the adjacent angle of a right angle triangle whose 
		hypotenuse is the distance between robot and target and adjacent side 
		
	Step2: Drive straight to the target
		We need to drive straight to the target with distance equal to the hypotenuse
		
	Step3: Turn to orient itself as expected
		The final angle to turn = expected angle - angle truned in step1
	
	'''




	x0 = robot.pose.position.x
	y0 = robot.pose.position.y

	ag = angle_z + robot.pose.rotation.angle_z.degrees

	var1=x+(x0*math.cos(math.radians(ag)))+(y0*math.sin(math.radians(ag)))
	var2=y+(y0*math.cos(math.radians(ag)))+(-x0*math.sin(math.radians(ag)))

	a = np.array([[math.cos(math.radians(ag)), math.sin(math.radians(ag))], [math.cos(math.radians(ag)), -math.sin(math.radians(ag))]])
	b = np.array([var1, var2])
	result = np.linalg.solve(a, b) #solving linear eqations to find global coordinates wref to relative coordinates

	x = result[0]
	y = result[1]


	hypotenuse = math.sqrt(((x)**2)+((y-y0)**2)) #hypotenuse gives the distance to travel
	adjacent = math.sqrt(((x-x0)**2)+(y0**2))

	tri_theta = math.degrees(math.acos(adjacent/hypotenuse)) #angle to turn so as to face the target
	theta = get_angle(x,y,tri_theta)

	my_turn_in_place(robot, theta, 10) #turn and face the target
	my_drive_straight(robot,hypotenuse,10)  #drive straight towards target
	my_turn_in_place(robot,(angle_z-theta),10) #turn towards the expected final orientation


def get_angle(x,y,angle):
	if(x>0 and y>0):
		return angle
	if(x>0 and y <0):
		return -angle
	if(x<0 and y>0):
		return (90+angle)
	if(x<0 and y< 0):
		return  (-90-angle)


def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""

	rho = math.inf


	thetag = math.radians(get_angle(x,y,angle_z))

	'''
	Converting relative coordinates to global coordinates in next few steps
	'''

	x0 = robot.pose.position.x
	y0 = robot.pose.position.y

	ag = angle_z + robot.pose.rotation.angle_z.degrees

	var1 = x + (x0 * math.cos(math.radians(ag))) + (y0 * math.sin(math.radians(ag)))
	var2 = y + (y0 * math.cos(math.radians(ag))) + (-x0 * math.sin(math.radians(ag)))

	a = np.array([[math.cos(math.radians(ag)), math.sin(math.radians(ag))],
				  [math.cos(math.radians(ag)), -math.sin(math.radians(ag))]])
	b = np.array([var1, var2])
	result = np.linalg.solve(a, b)  # solving linear eqations to find global coordinates wref to relative coordinates

	xg = result[0]
	yg = result[1]

	while rho>16: #stop when close to the target
		xr = robot.pose.position.x
		yr = robot.pose.position.y
		thetar = robot.pose.rotation.angle_z.radians ##

		print(xr)
		print(yr)
		print(thetar)

		#Compute the error in desired pose using equaltion 3.65
		rho = math.sqrt(((xr-xg)**2)+((yr-yg)**2))
		alpha = thetar - (math.atan((yr-yg)/(xr-xg))) ##
		neta = thetag - thetar

		print("Error")

		print(rho)
		print(alpha)
		print(neta)

		#setting hyperparamateres p1, p2, p3
		p1= 0.05
		p2 =0.004
		p3 = 0.08

		#find x_dot and theta_dot using equaltions 3.66 and 3.67
		x_dot = p1*rho
		theta_dot = ((p2*alpha) + (p3*neta))

		print("Dot values")

		print(x_dot)
		print(theta_dot)

		'''
		since linear velocity = angualr velocaity * radius we elimiate the radius from the equaltion in 3.64		
		'''

		phi_left = (( x_dot) - (theta_dot *get_front_wheel_radius()))
		phi_right = (( x_dot) + (theta_dot *get_front_wheel_radius()))

		print("Speed")

		print(phi_left)
		print(phi_right)
		robot.drive_wheels((phi_left), (phi_right))
		time.sleep(1)

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""

	angle = get_angle(x,y,angle_z)
	'''
	If the target is behing the robot, turn the bot to face in that direction
	'''
	if((abs(angle))> abs(angle_z)):
		my_turn_in_place(angle,20)

	'''
	Move towards target using pose2 method
	'''
	my_go_to_pose2(x,y,angle_z)
	pass

def run(robot: cozmo.robot.Robot):
	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))
	
	cozmo_drive_straight(robot, 62, 50)
	cozmo_turn_in_place(robot, 60, 30)
	cozmo_go_to_pose(robot, 100, 100, 45)

	rotate_front_wheel(robot, 90)
	my_drive_straight(robot, 62, 50)
	my_turn_in_place(robot, 90, 30)

	my_go_to_pose1(robot, 100, 100, 45)
	my_go_to_pose2(robot, 100, 100, 45)
	my_go_to_pose3(robot, 100, 100, 45)




if __name__ == '__main__':

	cozmo.run_program(run)



