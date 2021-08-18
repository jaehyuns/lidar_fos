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
	

	def circles_4_center(self,sorted_list):
		sorted_2_list=None
		sorted_2_list=sorted(sorted_list,key= lambda circle:circle.center.y)
		return sorted_2_list
	def get_center(self, obstacles):
		self.obData = obstacles	
	
	
	def calcEquidistance(self,x1, x2, x3, y1, y2, y3):
    		xmAB = (x1 + x2) / 2
		ymAB = (y1 + y2) / 2
		xmBC = (x3 + x2) / 2
	 	ymBC = (y3 + y2) / 2
		xmAC = (x3 + x1) / 2
		ymAC = (y3 + y1) / 2

	    	yDiffAB = y2 - y1
	    	xDiffAB = x2 - x1
	    	yDiffBC = y3 - y2
	    	xDiffBC = x3 - x2
	    	yDiffAC = y3 - y1
	    	xDiffAC = x3 - x1

    		if yDiffAB != 0 and xDiffAB != 0:
        		a = -1 / (yDiffAB / xDiffAB)
        		a1 = ymAB - (a*xmAB)

    		if yDiffBC != 0 and xDiffBC != 0:
        		b = -1 / (yDiffBC / xDiffBC)
        		b1 = ymBC - (b*xmBC)

    		if yDiffAC != 0 and xDiffAC != 0:
        		c = -1 / (yDiffAC / xDiffAC)
        		c1 = ymAC - (c*xmAC)

    		if yDiffAB != 0 and xDiffAB != 0 and yDiffBC != 0 and xDiffBC != 0 and yDiffAC != 0 and xDiffAC != 0:
        		self.center_x = -((a1 + (-b1)) / ((-b) + a))
        		self.center_y= (b * self.center_x) + b1

        		print("\nPonto equidistante:", [self.center_x, self.center_y])

			return self.center_x,self.center_y

    		elif yDiffAB != 0 and xDiffAB != 0 and yDiffBC != 0 and xDiffBC != 0:
        		self.center_x = -((a1 + (-b1)) / ((-b) + a))
        		self.center_y = (b * self.center_x) + b1

        		print("\nPonto equidistante:", [self.center_x, self.center_y])

			return self.center_x,self.center_y


    		elif yDiffBC != 0 and xDiffBC != 0 and yDiffAC != 0 and xDiffAC != 0:
        		self.center_x = -((b1 + (-c1)) / ((-c) + b))
        		self.center_y = (c * self.center_x) + c1

        		print("\nPonto equidistante:", [self.center_x, self.center_y])

			return self.center_x,self.center_y


    		elif yDiffAB != 0 and xDiffAB != 0 and yDiffAC != 0 and xDiffAC != 0:
        		self.center_x = -((c1 + (-a1)) / ((-a) + c))
        		self.center_y = (a * self.center_x) + a1

        		print("\nPonto equidistante:", [self.center_x, self.center_y])
	
			return self.center_x,self.center_y

	

	def circle(self):
		print("CNT:",len(self.obData.circles))
		if len(self.obData.circles) == 0: 
			self.xycar_angle_deg = 0
			print("zero2")
			drive(self.xycar_angle_deg,4)

		elif len(self.obData.circles) == 1:
			circles=self.obData.circles
			if(circles[0].center.y > 0.0003):
				drive(-25,3)
				print("aa")	
			elif(circles[0].center.y < 0.0003):
				drive(25,3)
				print("bb")


		else:
			circles=self.obData.circles # List of Circles Data
			#print("Circle:",self.obData.circles[0].center.x)
			left_circle=None
			right_circle=None
			sorted_list=None
			a=1.4
	                b=0.7
			if(len(circles)>2):
				sorted_list=sorted(circles,key= lambda circle:circle.center.x) # near(x-axis) 2 Circle
				if(len(circles)==3):
					sorted_2_list=self.circles_4_center(sorted_list)
					filter_point1=sorted_2_list[0].center
					filter_point2=sorted_2_list[1].center
					filter_point3=sorted_2_list[2].center
					

					self.center_x,self.center_y=self.calcEquidistance(filtered_point1.x,filtered_point2.x,filtered_point3.x,filtered_point1.y,filtered_point2.y,filtered_point3.y)
					self.calc_angle()
					drive(self.xycar_angle_deg,4)
					

					#if(filter_point3.y<0):
					#	drive(18,3)
						
					#else:
					#	if(filter_point2.y<0):
					#		drive(13,3)					
					#	else:
					#		drive(-13,3)
					

						
								


				if(len(circles)>3):
					sorted_2_list=self.circles_4_center(sorted_list)					
					left_circle_1=sorted_2_list[0]
					left_circle_2=sorted_2_list[1]
					right_circle_1=sorted_2_list[2]
					right_circle_2=sorted_2_list[3]
					left_point1=left_circle_1.center
					left_point2=left_circle_2.center
					right_point1=right_circle_1.center
					right_point2=right_circle_2.center
					self.center_x=(left_point1.x+left_point2.x+right_point1.x+right_point2.x)/4
					self.center_y=(left_point1.y+left_point2.y+right_point1.y+right_point2.y)/4
					self.calc_angle()
					drive(self.xycar_angle_deg,4)
					return
				
				
			
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
                                        

					
				
				if(right_circle.center.y<0):
					self.xycar_angle_deg=20	
 					drive(self.xycar_angle_deg,3)
					
					return
				elif(left_circle.center.y>0):
					self.xycar_angle_deg=-20
					drive(self.xycar_angle_deg,3)
					
					return
					
				print("left_circle:",left_circle.center.y,"right_circle:",right_circle.center.y)

					


			left_point=left_circle.center
			right_point=right_circle.center
			self.center_x=(left_point.x+right_point.x)/2
			self.center_y=(left_point.y+right_point.y)/2
			self.calc_angle()
		
			print("x:",self.center_x,"y:",self.center_y,"deg:",self.xycar_angle_deg)
		        drive(self.xycar_angle_deg,4)

			#self.publish_angle()
			

	def calc_angle(self):
		self.xycar_angle = math.atan2(self.center_y,self.center_x)
		self.xycar_angle_deg = self.xycar_angle*180/math.pi #obstacles chase
		if (self.xycar_angle_deg > 30): self.xycar_angle_deg = 30
		elif (self.xycar_angle_deg < -30): self.xycar_angle_deg = -30
		print("Angle deg : ", self.xycar_angle_deg)


if __name__=='__main__':
	ob=point()
	rospy.Subscriber("/obstacles",Obstacles, ob.get_center, queue_size=1)      
	rospy.init_node('test3')  	
	#srv = Server(dynamic_testConfig, callback)
	time.sleep(1)
	while not rospy.is_shutdown():
		time.sleep(1)
		ob.circle()
	
	print('Done')
	
