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
RECOV = 1.5
Speed = 0.1
def drive(angle, speed):
	global drive_values_pub
	global Speed
	drive_value = drive_values()
	drive_value.throttle = Speed
	drive_value.steering = angle

	#drive_values_pub.publish(drive_value)

	print("steer : ", drive_value.steering*180/math.pi)
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

	

	def segment(self):
		list_l_y=[]
		list_r_y=[]
		list_x=[]
		global RECOV
		if len(self.obData.segments) == 0: 
			self.xycar_angle_deg = 0
			#self.publish_angle()
			return
		for seg in self.obData.segments:
			fp = seg.first_point # first_point
			lp = seg.last_point # last_point
			if fp.y < -0.001 : 
				list_l_y.append(fp.y)
			elif fp.y > 0.001 :
				list_r_y.append(fp.y) 			
			list_x.append(fp.x)
			list_x.append(lp.x)

		if len(list_l_y) == 0 and len(list_r_y) == 0:
			self.xycar_angle = 0
			#self.publish_angle()
			return

		elif len(list_l_y) == 0 and sum(list_r_y)/len(list_r_y) <= 0.1:
			RECOV = 3
			list_l_y = [-i*RECOV for i in list_r_y]
			print("aa")

		elif len(list_l_y) == 0 and sum(list_r_y)/len(list_r_y) > 0.1:
			RECOV = 1.5 
			list_l_y = [-i*RECOV for i in list_r_y]
			print("bb")	

		elif len(list_r_y) == 0 and sum(list_l_y)/len(list_l_y) >= -0.1:
			RECOV = 3
			list_r_y = [-i*RECOV for i in list_l_y]
			print("cc")

		elif len(list_r_y) == 0 and sum(list_l_y)/len(list_l_y) < -0.1:
			RECOV = 1.5 
			list_r_y = [-i*RECOV for i in list_l_y]
			print("dd")

		self.center_y = (sum(list_l_y)/len(list_l_y) + sum(list_r_y)/len(list_r_y)) / 2.0
		# self.center_x = sum(list_x)/len(list_x)
		self.center_x = sum(list_x)/len(list_x)

		print("L_y : ", sum(list_l_y)/len(list_l_y))
		print("R_y : ", sum(list_r_y)/len(list_r_y))
		print("x : ", sum(list_x)/len(list_x))
		print("CNT : ", len(self.obData.segments))
		print("End! \n")
		self.calc_angle()
                drive(self.xycar_angle, 0)

		#self.publish_angle()
		

	def calc_angle(self):
		self.xycar_angle = math.atan2(self.center_y,self.center_x)
		#self.xycar_angle_deg = self.xycar_angle*180/math.pi #obstacles chase
		#if (self.xycar_angle_deg > 26): self.xycar_angle_deg = 26
		#elif (self.xycar_angle_deg < -26): self.xycar_angle_deg = -26
		print("Angle Rad : ", self.xycar_angle)

#def callback(config, level):
	    
 	#global RECOV
	#RECOV = config.RECOV
	#rospy.loginfo("""Reconfigure Request: {RECOV} ,""".format(**config))
	#return config


if __name__=='__main__':
	ob=point()
	rospy.Subscriber("/obstacles",Obstacles, ob.get_center, queue_size=1)      
	rospy.init_node('test3')  	
	#srv = Server(dynamic_testConfig, callback)
	time.sleep(3)
	while not rospy.is_shutdown():
		#ob.circle()
		ob.segment()
		time.sleep(1)


	print('Done')
	
