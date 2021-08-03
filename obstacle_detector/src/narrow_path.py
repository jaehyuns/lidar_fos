#!/usr/bin/env python 

import rospy
import math, time

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
# from xycar_motor.msg import xycar_motor



class narrowPath:
	
	def __init__(self):
		rospy.init_node('test_path')

		self.pub = rospy.Publisher('ackermann_cmd',AckermannDriveStamped, queue_size=10)
		# self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)		
		self.obData = None
		self.center_x = None
		self.center_y = None
		self.xycar_angle = None
		self.xycar_angle_deg = None

	def get_center(self, obstacles):
		self.obData = obstacles	

	def circle(self):
		list_x=[]
		list_y=[]
		if len(self.obData.circles) == 0:
			self.xycar_angle_deg = 0
			self.publish_angle()
			return
		for circle in self.obData.circles:
			p = circle.center
			list_x.append(p.x)
			list_y.append(p.y)
		self.center_x = sum(list_x)/len(list_x)
		self.center_y = sum(list_y)/len(list_y)
		print("X : ", sum(list_x)/len(list_x))
		print("Y : ", sum(list_y)/len(list_y))
		print("CNT : ", len(list_x))
		print("End! \n")
		self.calc_angle()
		self.publish_angle()
		
	def segment(self):
		list_l_x=[]
		list_r_x=[]
		list_y=[]

		if len(self.obData.segments) == 0:
			self.xycar_angle_deg = 0
			self.publish_angle()
			return
		for seg in self.obData.segments:
			fp = seg.first_point # first_point
			lp = seg.last_point # last_point
			if fp.x < -0.001 : 
				list_l_x.append(fp.x)
			elif fp.x > 0.001 :
				list_r_x.append(fp.x) 			
			list_y.append(fp.y)
			list_y.append(lp.y)

		if len(list_l_x) == 0 and len(list_r_x) == 0:
			self.xycar_angle_deg = 0
			self.publish_angle()
			return

		elif len(list_l_x) == 0:
			list_l_x = [-i*1.5 for i in list_r_x]
		elif len(list_r_x) == 0:
			list_r_x = [-i*1.5 for i in list_l_x]
		

		self.center_x = (sum(list_l_x)/len(list_l_x) + sum(list_r_x)/len(list_r_x)) / 2.0
		# self.center_x = sum(list_x)/len(list_x)
		self.center_y = sum(list_y)/len(list_y)

		print("L_X : ", sum(list_l_x)/len(list_l_x))
		print("R_X : ", sum(list_r_x)/len(list_r_x))
		print("Y : ", sum(list_y)/len(list_y))
		print("CNT : ", len(self.obData.segments))
		print("End! \n")
		self.calc_angle()
		self.publish_angle()
		

	def calc_angle(self):
		self.xycar_angle = math.atan(self.center_x/self.center_y)
		self.xycar_angle_deg = self.xycar_angle*180/math.pi #obstacles chase
		if (self.xycar_angle_deg > 26): self.xycar_angle_deg = 26
		elif (self.xycar_angle_deg < -26): self.xycar_angle_deg = -26
		print("Angle Rad : ", self.xycar_angle)
		
	def publish_angle(self):
		acker_data = AckermannDriveStamped()
		acker_data.drive.speed = 0 
		acker_data.drive.steering_angle = self.xycar_angle
		self.pub.publish(acker_data)
		"""
		motor_msg = xycar_motor()
		motor_msg.header.stamp = rospy.Time.now()
		motor_msg.angle = self.xycar_angle_deg
		motor_msg.speed = 3
		self.pub.publish(motor_msg)"""



if __name__=='__main__':
	ob=narrowPath()
	rospy.Subscriber("/obstacles",Obstacles, ob.get_center, queue_size=1)
	

	time.sleep(3)
	while not rospy.is_shutdown():
		#ob.circle()
		ob.segment()
		time.sleep(1)


	print('Done')
