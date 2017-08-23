#!/usr/bin/env python
'''
Program to locate a person in frame, and maintain a specified distance away while the program is running.
'''
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time as timer
import cv2
from PIL import Image
from sensor_msgs.msg import Image as imageX
import ros_numpy
from __future__ import print_function
from imutils.object_detection import non_max_suppression
import imutils

rospy.init_node('takeoff', anonymous=True)
pubT = rospy.Publisher("ardrone/takeoff", Empty, queue_size=1)
commandDrone= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
pubL = rospy.Publisher("ardrone/land", Empty, queue_size=1)
rate = rospy.Rate(10) # 10hz

def control():
	SendCommand(0,0,0,0)
	takeoff()
	rospy.sleep(5)
	SendCommand(0,0,0,0.75)
	rospy.sleep(0.5)
	SendCommand(0,0,0,0)
	rospy.sleep(1)
	z=True
	time=0
	while(z==True):
		#frame=saveImg()
		xCoord,yCoord, width, length, f= findFace()
		print "Face found. Enabling tracking algorithm.."
		track(xCoord,yCoord, width, length, f)
		#else:
		#	z=False
	rospy.sleep(2)
	land()
	print "done."

def findFace():
	param=True
	ti=0
	var=0
	r=0
	while (param):
		xC=0
		yC=0
		width=0
		face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
		msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
		img = ros_numpy.numpify(msg)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, 1.3, 5)
		for (x,y,w,h) in faces:
			xC=x
			yC=y
			width=w
		
		if (xC==0):
			print "hello"
		else:
			print "Face found..."
			param=False
			SendCommand(0,0,0,0)
	return xC, yC, width, width, img

def track(x,y,w,h,f):
    #msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
    #f = ros_numpy.numpify(msg)
    tracker = cv2.Tracker_create("KCF")
    x=x-w
    y=y-w
    xC=(int) (x-1.5*w)
    yC=(int) (y-0.75*w)
    height=h*11
    width= (int) (w*3.5)
    c,r,w,h=xC,yC,width,height
    bbox = (c,r,w,h)
    ok = tracker.init(f, bbox)
    num=0#once we get to looking for distance every 5 frames
    var=0
    x=0
    while True:
    	x+=1
        msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
        frame = ros_numpy.numpify(msg)
        ok=True
        ok, bbox = tracker.update(frame)
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0,0,255))
        cv2.imshow("Tracking", frame)
        angle= findAngle((p1[0]+p2[0])/2)
        #update distance
        if(var==5):
        	findDistance()
        	var=0
        else:
        	var+=1
        #update angle
        if(angle<=5 and angle>=-5 and var>0):
            SendCommand(0,0,0,0)
            var=0
        elif(angle>5 and var==0):
            SendCommand(0,0,-0.3,0)
            var+=1
        elif(angle<-5 and var==0):
            SendCommand(0,0,0.3,0)
            var+=1
        if(x==1):
        	ratio= initDist(frame, p2[0]-p1[0],p2[1]-p1[1])
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break

def findAngle(xCoord):
	return round((xCoord-320)*35/320)

def initDist(frame,width,height):
	hog = cv2.HOGDescriptor()
	hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
	image = imutils.resize(frame, width=min(400, image.shape[1]))
	orig = image.copy()
	(rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
		padding=(8, 8), scale=1.05)
	for (x, y, w, h) in rects:
		cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
	rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
	pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
	for (xA, yA, xB, yB) in pick:
		cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
	cv2.imshow("Before NMS", orig)
	cv2.imshow("After NMS", image)

def findDistance(frame,width,height):


def saveImg():
    msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
    frame = ros_numpy.numpify(msg)
    return frame
    #imageOpener("droneIm"+str(x)+".jpg")

def takeoff():
	print "Taking off..."
	for x in range(0,5):
		pubT.publish(Empty())
		rate.sleep()

def land():
	print "Landing..."
	for x in range(0,5):
		pubL.publish(Empty())
		rate.sleep()

def SendCommand(roll,pitch,yaw_velocity,z_velocity):
	command=Twist()
	command.linear.x = pitch
	command.linear.y = roll
	command.linear.z = z_velocity
	command.angular.z = yaw_velocity
	commandDrone.publish(command)

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
