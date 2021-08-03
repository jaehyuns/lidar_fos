#!/usr/bin/env python 

import rospy
import math, time

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped


class avoidObstacles:
	def __init__(self):
		rospy.init_node('test_path1')
		self.pub = rospy.Publisher('ackermann_cmd',AckermannDriveStamped, queue_size=10)
		rospy.Subscriber("/obstacles",Obstacles, self.get_center_avoid, queue_size=1)

		self.avoid=None
		self.center_x=None
		self.center_y=None
	def get_center_avoid(self,avoid):
		self.avoid=avoid
		
	def circle(self):
		list_x=[]
		list_y=[]
		if len(self.avoid.circles) == 0:
			self.xycar_angle_deg = 0
			self.publish_angle()
			return

		for circle in self.avoid.circles:
			p=circle.center
			list_x.append(p.x)
			list_y.append(p.y)
						
		self.center_x=sum(list_x)/len(list_x)
		self.center_y=sum(list_y)/len(list_y)
		print("X : ", sum(list_x)/len(list_x))
		print("Y : ", sum(list_y)/len(list_y))
		print("CNT : ", len(list_x))
		print("End! \n")
		self.calc_angle()
		self.publish_angle()

	def calc_angle(self):
		self.xycar_angle = math.atan(self.center_x/self.center_y)
		self.xycar_angle_deg = self.xycar_angle*180/math.pi #obstacles chase
		if (self.xycar_angle_deg > 26): self.xycar_angle_deg = 26
		elif (self.xycar_angle_deg < -26): self.xycar_angle_deg = -26
		print("Angle Deg : ", self.xycar_angle_deg)
		
	def publish_angle(self):
		acker_data = AckermannDriveStamped()
		acker_data.drive.speed = 0 
		acker_data.drive.steering_angle = -self.xycar_angle_deg * 0.1
		self.pub.publish(acker_data)
		
if __name__=='__main__':
		
	ob=avoidObstacles()
	
	time.sleep(3)
	while not rospy.is_shutdown():
		ob.circle()
		#ob.segment()
		time.sleep(1)


	print('Done')
