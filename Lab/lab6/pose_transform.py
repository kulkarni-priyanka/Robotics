#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy as np
from cozmo.util import degrees
import math

def get_relative_pose(object_pose, refrence_frame_pose):

	'''
	 values x,y and angle are derived using the rotational and translational transform equations
	'''

	angle = object_pose.rotation.angle_z - refrence_frame_pose.rotation.angle_z

	x0 =refrence_frame_pose.position.x
	y0 = refrence_frame_pose.position.y
	x1 = object_pose.position.x
	y1 = object_pose.position.y
	a = refrence_frame_pose.rotation.angle_z.radians

	x = (x1-x0) * math.cos(a) + (y1-y0) * math.sin(a)
	y = (y1-y0) * math.cos(a) - (x1-x0) * math.sin(a)

	final = cozmo.util.Pose(x,y,refrence_frame_pose.position.z,angle_z=angle)
	return final

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose: %s" % cube.pose)
				final = get_relative_pose(cube.pose, robot.pose)
				print("Cube pose in the robot coordinate frame: %s" % final)
				robot.go_to_pose((final), relative_to_robot=True).wait_for_completed()

		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
