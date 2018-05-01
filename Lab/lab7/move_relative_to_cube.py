#!/usr/bin/env python3

'''
This is starter code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys
from Lab.lab6 import pose_transform
from Lab.lab7 import odometry
import asyncio

def move_relative_to_cube(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, when a cube is detected it
	moves the robot to a given pose relative to the detected cube pose.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while cube is None:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Found a cube, pose in the robot coordinate frame: %s" % pose_transform.get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")

	desired_pose_relative_to_cube = Pose(0, 100, 0, angle_z=degrees(90))
	desired_x = desired_pose_relative_to_cube.position.x + cube.pose.position.x
	desired_y = desired_pose_relative_to_cube.position.y + cube.pose.position.y
	desired_angle = desired_pose_relative_to_cube.rotation.angle_z.degrees + cube.pose.rotation.angle_z.degrees
	desired = Pose(desired_x, desired_y, 0, angle_z=degrees(desired_angle))
	final = pose_transform.get_relative_pose(desired, robot.pose)
	print(robot.pose)
	print(cube.pose)
	print(final)

	odometry.my_go_to_pose1(robot, final.position.x, final.position.y, final.rotation.angle_z.degrees)




if __name__ == '__main__':

	cozmo.run_program(move_relative_to_cube)