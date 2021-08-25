#!/usr/bin/env python

import rospy
import math, time

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from race.msg import drive_values
from math import atan2, degrees
#from dynamic_reconfigure.server import Server
#from obstacle_detector.cfg import dynamic_testConfig
# from ackermann_msgs.msg import AckermannDriveStamped
# from xycar_motor.msg import xycar_motor


drive_values_pub = rospy.Publisher('control_value', drive_values, queue_size=1)

def cal_distance(x,y):
	return (math.sqrt(x**2+y**2))

def drive(angle, speed):
	global drive_values_pub
	
	drive_value = drive_values()
	drive_value.steering = angle
	if (drive_value.steering>23):
		drive_value.throttle=3
		drive_value.steering = 30.0
	elif (drive_value.steering<-23):
		drive_value.throttle=3
		drive_value.steering=-30.0
	else:		
		drive_value.throttle = speed

	

	drive_values_pub.publish(drive_value)

	print("steer : ", drive_value.steering)
	print("throttle : ", drive_value.throttle)
class point:
	def __init__(self):
		self.distance_min=None
		self.obData = None
		self.center_x = None
		self.center_y = None
		self.xycar_angle = None
		self.xycar_angle_deg = None
		self.angle=None
	
	
	def avoid_collision1(self,x,y,steer):
		dis=cal_distance(x,y)
		print("dis",dis)
		if(dis<0.5):
			if(steer>0):
				print("Absol,right")
				self.center_y+=0.5
			elif(steer<0):
				self.center_y-=0.3
				print("Absol,left")
	def avoid_collision(self,distance_min,steer):
	
		print("distance_min",distance_min)
		if(distance_min<0.5):
			if(steer>0):
				print("Absol,right1111111111111")
				self.center_y+=0.5
			elif(steer<0):
				self.center_y-=0.3
				print("Absol,left11111111111111111111")	
				
				
		
	def circles_4_center(self,sorted_list):
		sorted_2_list=None
		sorted_2_list=sorted(sorted_list,key= lambda circle:circle.center.y)
		return sorted_2_list
	def get_center(self, obstacles):
		self.obData = obstacles	
	
	
	def calc_dismin(self,x1, x2, x3, x4, y1, y2, y3, y4):
		
		dis1=math.sqrt(x1**2+y1**2)
		print("dis1, ",dis1)
		dis2=math.sqrt(x2**2+y2**2)
		print("dis2, ",dis2)
		dis3=math.sqrt(x3**2+y3**2)
		print("dis3, ",dis3)
		dis4=math.sqrt(x4**2+y4**2)
		print("dis4, ",dis4)
		dis_min=min(dis1,dis2,dis3,dis4)
		return dis_min

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



	def angle_between(self,x1, x2, x3, y1, y2, y3):
		
		
		deg1=(360+degrees(atan2(x1-x2,y1-y2)))%360
		deg2=(360+degrees(atan2(x3-x2,y3-y2)))%360
		
		return deg2-deg1 if deg1<=deg2 else 360-(deg1-deg2)

		

	def circle(self):
		print("CNT:",len(self.obData.circles))
		if len(self.obData.circles) == 0: 
			self.xycar_angle_deg = 0
			print("zero2")
			drive(self.xycar_angle_deg,5)

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
			sorted_list=[1,2,3,4]
			sorted_list_3=[1,2,3]
			a=1.4
	                b=0.3
			if(len(circles)>2):
	
				if(len(circles)==3):
					sorted_first_list=sorted(circles,key= lambda circle:circle.center.x)
					sorted_list_3[0]=sorted_first_list[0]
					sorted_list_3[1]=sorted_first_list[1]
					sorted_list_3[2]=sorted_first_list[2]


					sorted_2_list=self.circles_4_center(sorted_list_3)
					
					filter_point1=sorted_2_list[0].center
					filter_point2=sorted_2_list[1].center
					filter_point3=sorted_2_list[2].center
				
					self.center_x,self.center_y=self.calcEquidistance(filter_point1.x,filter_point2.x,filter_point3.x,filter_point1.y,filter_point2.y,filter_point3.y)
					self.calc_angle()
					drive(self.xycar_angle_deg,4)
					
						
								


				else:
					
					sorted_first_list=sorted(circles,key= lambda circle:circle.center.x)
					sorted_list[0]=sorted_first_list[0]
					sorted_list[1]=sorted_first_list[1]
					sorted_list[2]=sorted_first_list[2]
					sorted_list[3]=sorted_first_list[3]


					sorted_2_list=self.circles_4_center(sorted_list)

									
					left_circle_1=sorted_2_list[0]
					left_circle_2=sorted_2_list[1]
					right_circle_1=sorted_2_list[2]
					right_circle_2=sorted_2_list[3]
					left_point1=left_circle_1.center
					left_point2=left_circle_2.center
					right_point1=right_circle_1.center
					right_point2=right_circle_2.center

						
					if (left_point2.y>0.01):
						self.angle=self.angle_between(left_point2.x,right_point1.x,right_point2.x,left_point1.y,right_point1.y,right_point2.y)
						if (self.angle>90):
							
							drive(30,4)
							print("3right")
							return
							
						else:
					
							self.center_x=(left_point1.x+left_point2.x+right_point1.x+right_point2.x)/4
							self.center_y=(left_point1.y+left_point2.y+right_point1.y+right_point2.y)/4 #b
							self.calc_angle()
							self.distance_min=self.calc_dismin(left_point1.x,left_point2.x,right_point1.x,right_point2.x,left_point1.y,left_point2.y,right_point1.y,right_point2.y)
							
							self.avoid_collision(self.distance_min,self.xycar_angle_deg)
							self.calc_angle()
							drive(self.xycar_angle_deg,4)
							print("normal path--1")
							return




					
					else:
					
						self.center_x=(left_point1.x+left_point2.x+right_point1.x+right_point2.x)/4
						self.center_y=(left_point1.y+left_point2.y+right_point1.y+right_point2.y)/4
						self.calc_angle()
						self.distance_min=self.calc_dismin(left_point1.x,left_point2.x,right_point1.x,right_point2.x,left_point1.y,left_point2.y,right_point1.y,right_point2.y)
						self.avoid_collision(self.distance_min,self.xycar_angle_deg)
						#self.avoid_collision(nearest_point.x,nearest_point.y,self.xycar_angle_deg)
						self.calc_angle()
						drive(self.xycar_angle_deg,4)
						print("normal path--2")
						return



						
						
			elif(len(circles)==2):
				
				if(circles[0].center.y<circles[1].center.y):
					left_circle=circles[0]
					right_circle=circles[1]
					
				else:
					left_circle=circles[1]
					right_circle=circles[0]
                                        

					
				
				if(right_circle.center.y<0):
					self.xycar_angle_deg=20	
 					drive(self.xycar_angle_deg,4)
					
					return
				elif(left_circle.center.y>0):
					self.xycar_angle_deg=-20
					drive(self.xycar_angle_deg,4)
					
					return
					
				print("left_circle:",left_circle.center.y,"right_circle:",right_circle.center.y)

					
				left_point=left_circle.center
				right_point=right_circle.center
				self.center_x=(left_point.x+right_point.x)/2
				self.center_y=(left_point.y+right_point.y)/2
				self.calc_angle()
			
				print("x:",self.center_x,"y:",self.center_y,"deg:",self.xycar_angle_deg)
				drive(self.xycar_angle_deg,4)

				self.publish_angle()
			

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
		time.sleep(0.1)
		ob.circle()
	
	print('Done')
