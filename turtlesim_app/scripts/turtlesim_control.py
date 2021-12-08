#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.srv import *
from std_srvs.srv import *
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
import sys
import time
PI = 3.1415926535897

def moveToTarget(distance, isForward):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    speed = 5

    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
    if not rospy.is_shutdown():
    	t0 = float(rospy.Time.now().to_sec())
    	current_distance = 0
    	#robotu belirlenmis mesafeye goturme islemi
    	while(float(current_distance) < float(distance)):
    		velocity_publisher.publish(vel_msg)
    		#gercek zamanin konuma cevrilmesi
    		t1=float(rospy.Time.now().to_sec())
    		current_distance= float(speed*(t1-t0))
    	#robotun hizi sifirlandi
    	vel_msg.linear.x = 0
    	#robot durduruldu
    	velocity_publisher.publish(vel_msg)
        
def rotate(angle, clockwise):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = 20 #aci/saniye

    #dereceyi radyana cevirme
    angular_speed = speed*2*PI/360
    relative_angle = float(float(angle)*2*PI/360)

    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # robot yonu kontrolu (saat yonu- saat yonu tersi)
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
   
    t0 = float(rospy.Time.now().to_sec())
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = float(rospy.Time.now().to_sec())
        current_angle = float(angular_speed*(t1-t0))


    #robot durduruldu.
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def move_circle(sec , speed, rot):
	pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size = 10)
	vel_msg = Twist()
	
	radius = 20

	vel_msg.linear.x = speed
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = float(rot)
	t0 = float(rospy.Time.now().to_sec())
	
	t1 = float(rospy.Time.now().to_sec())
	while(float(t1-t0) <= float(sec)):
		pub.publish(vel_msg)
		t1 = float(rospy.Time.now().to_sec())
	
	vel_msg.linear.x = 0	
	vel_msg.angular.z = 0
	pub.publish(vel_msg)	

    
def drawA():
	rotate(70, 0)
	moveToTarget(4, 1)
	rotate(140, 1)
	moveToTarget(4, 1)
	moveToTarget(2, 0)
	rotate(109.85, 1)	
	moveToTarget(1.35, 1)
	killTurtle("turtle1")
	time.sleep(5)
	clearStage()	
	spawnTurtle(4,5.544445,0,"turtle1")
	
	
def drawB():
	rotate(90, 0)
	moveToTarget(4,1)
	rotate(90, 1)
	moveToTarget(1,1)
	move_circle(0.777, 4, -4)
	moveToTarget(0.9,1)
	rotate(180,1)
	moveToTarget(1.34,1)
	move_circle(0.777, 4, -4)
	moveToTarget(1.24,1)
	killTurtle("turtle1")
	time.sleep(5)
	clearStage()	
	spawnTurtle(4,5.544445,0,"turtle1")
    
def poseCallback(data):
    rospy.loginfo("pose x: " + str(data.x) + "pose y: " + str(data.y) + "pose theta: " + str(data.theta))
    
if __name__ == '__main__':
	try:
		#ros icin gerekli olan temel fonksiyonlarin olusturulmasi
		rospy.init_node('draw_letter', anonymous=True)
		killTurtle = rospy.ServiceProxy('/kill', Kill)
		spawnTurtle = rospy.ServiceProxy('/spawn', Spawn)
		clearStage = rospy.ServiceProxy('/clear', Empty)
		moveToTarget(1,1)
		killTurtle("turtle1")
		spawnTurtle(4,5.544445,0,"turtle1") #belirtilen konumda yeni turtle olusturma.
		
		# Ornek Twist publisher ve Pose subscriber
		# pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		# C'den farkli olarak pythonda subscriber ayri bir thread olarak program kapanana kadar calisir
		# Dolayisiyla bu kisim yorumdan kalkarsa callback fonksiyonu surekli olarak konumu basar
		# rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
		# rospy.spin()
		rate = rospy.Rate(10)
		
		
		while True:
                        
			print("bir harf giriniz (A,B), cikis icin '.' giriniz:")
			
			val = input()
			if val == 'A' or val == 'a':
				drawA()
			elif val == 'B' or val == 'b':
				drawB()
			elif val == '.':
				sys.exit()
			else:
				print("yanlis harf")
				
			rate.sleep()
			# Tekil mesaj okumalarinda wait_for_message fonksiyonu alternatif olabilir.
			# Bu uygulamada sadece her hareketten sonra konum bilgisine ihtiyac oldugundan
			# wait_for_message fonksiyonu cagrimi yapilmistir.
			# Fakat odevde surekli olarak konum bilgisine ihtiyac olacagindan
			# subscribe uzerinden callback cagrimi daha uygundur.
			msg = rospy.wait_for_message('/turtle1/pose', Pose, timeout=None)
			poseCallback(msg)
				
	except rospy.ROSInterruptException: pass
