#!/usr/bin/env python

import rospy
import math, time

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from race.msg import drive_values
#from dynamic_reconfigure.server import Server
#from obstacle_detector.cfg import dynamic_testConfig
# from ackermann_msgs.msg import AckermannDriveStamped
# from xycar_motor.msg import xycar_motor


drive_values_pub = rospy.Publisher('control_value', drive_values, queue_size=1)


def cal_distance(x,y):
	return (sqrt(x**2+y**2))

def drive(angle, speed):
	global drive_values_pub
	
	drive_value = drive_values()
	drive_value.throttle = speed
	drive_value.steering = angle

	drive_values_pub.publish(drive_value)

	print("steer : ", drive_value.steering)
	print("throttle : ", drive_value.throttle)

class point:
	def __init__(self):
		
		self.obData = None
		self.center_x = None
		self.center_y = None
		self.xycar_angle = None
		self.xycar_angle_deg = None

	def get_center(self, obstacles):
		self.obData = obstacles	
		
	

	def circle(self):
		print("CNT:",len(self.obData.circles))
		if len(self.obData.circles) == 0: 
			self.xycar_angle_deg = 0
			print("zero2")
			drive(self.xycar_angle_deg,4)
		else:
			circles=self.obData.circles # List of Circles Data
			#print("Circle:",self.obData.circles[0].center.x)
			left_circle=None
			right_circle=None
			sorted_list=None

		
			if(len(circles)>2):
				sorted_list=sorted(circles,key= lambda circle:circle.center.x) # near(x-axis) 2 Circle
			
				filtered_circle1=sorted_list[0]
				filtered_circle2=sorted_list[1]
				if(filtered_circle1.center.y<filtered_circle2.center.y):
					left_circle=filtered_circle1
					right_circle=filtered_circle2
				else:
					left_circle=filtered_circle2
					right_circle=filtered_circle1
				
										
						
			elif(len(circles)==2):
				if(circles[0].center.y<circles[1].center.y):
					left_circle=circles[0]
					right_circle=circles[1]
				else:
					left_circle=circles[1]
					right_circle=circles[0]
				print("left_circle:",left_circle.center.y,"right_circle:",right_circle.center.y)
			left_point=left_circle.center
			right_point=right_circle.center
			self.center_x=(left_point.x+right_point.x)/2
			self.center_y=(left_point.y+right_point.y)/2
			self.calc_angle()
		
			print("x:",self.center_x,"y:",self.center_y,"deg:",self.xycar_angle_deg)
		        drive(self.xycar_angle_deg, 3)

			#self.publish_angle()
			

	def calc_angle(self):
		self.xycar_angle = math.atan2(self.center_y,self.center_x)
		self.xycar_angle_deg = self.xycar_angle*180/math.pi #obstacles chase
		if (self.xycar_angle_deg > 30): self.xycar_angle_deg = 30
		elif (self.xycar_angle_deg < -30): self.xycar_angle_deg = 30
		print("Angle deg : ", self.xycar_angle_deg)


if __name__=='__main__':
	ob=point()
	rospy.Subscriber("/obstacles",Obstacles, ob.get_center, queue_size=1)      
	rospy.init_node('test3')  	
	#srv = Server(dynamic_testConfig, callback)
	time.sleep(3)
	while not rospy.is_shutdown():
		time.sleep(1)
		ob.circle()
	
	print('Done')
	
