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
from imutils.object_detection import non_max_suppression
import imutils
import numpy as np

#initializes ros node and publishers
rospy.init_node('takeoff', anonymous=True)
pubT = rospy.Publisher("ardrone/takeoff", Empty, queue_size=1)
commandDrone= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
pubL = rospy.Publisher("ardrone/land", Empty, queue_size=1)
rate = rospy.Rate(10) # 10hz

def control():
	#commands drone to takeoff, go up for 0.5 seconds then stop
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
		#loads pretrained classifier
		face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
		#gets image from drone
		msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
		#converts raw image to numpy array
		img = ros_numpy.numpify(msg)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		#detects faces in frame using classifier
		faces = face_cascade.detectMultiScale(gray, 1.2, 5)
		for (x,y,w,h) in faces:
			xC=x
			yC=y
			width=w
		
		if (xC==0):
			print "Face not found..."
		else:
			print "Face found..."
			param=False
			SendCommand(0,0,0,0)
	return xC, yC, width, width, img

def track(x,y,w,h,f):
    trackerVar=0
    varia=str(trackerVar)
    #creates tracker
    varia = cv2.Tracker_create("KCF")
    x=x
    y=y
    #creates bounding box
    xC=(int) (x-1.5*w)
    yC=(int) (y-0.75*w)
    height=h*5
    width= (int) (w*3.5)
    c,r,w,h=xC,yC,width,height
    bbox = (c,r,w,h)
    #intialized tracker with the created bounding box
    ok = varia.init(f, bbox)
    ratioX,ratioY= initDist(f, width,height)
    num=0#once we get to looking for distance every 5 frames
    var=0
    turn=0
    forward=0
    nonLocate=0
    while True:
        msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
        frame = ros_numpy.numpify(msg)
        ok=True
	#updates position of bounding box using the KCF tracker
        ok, bbox = varia.update(frame)
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
	    #draws bounding  box on the current frame
            cv2.rectangle(frame, p1, p2, (0,0,255))
        if(num==3):
            #check initDistance for descr of pedestrian detector
            position=0
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            image = imutils.resize(frame, width=min(400, frame.shape[1]))
            orig = image.copy()
            (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
                padding=(8, 8), scale=1.05)
            for (x, y, w, h) in rects:
                cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
            rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
            pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
            par=0
            for (xA, yA, xB, yB) in pick:
                par+=1
                cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
            if(par>0):
		#checks if bounding box is close enough using pedestrian detector
                if(abs(((xB+xA)/2)-(p1[0]+p2[0])/2)>120 or abs(((yB+yA)/2)-((p1[1]+p2[1])/2))>40):
                 cv2.circle(image,(((xB+xA)/2),((yB+yA)/2)),2,(0,0,255),3)
                 cv2.circle(image,(p1[0],p1[1]),2,(0,255,0),3)
                 print "fixing bbox"
                 xC=xB
                 yC=yA
                 #height=h*5
                 #width= (int) (w*3.5)
		 #creates new bounding box and creates/initializes a new tracker
                 c,r,w,h=xC+width/2,yC+height/2,width,height
                 bbox = (c,r,w,h)
                 trackerVar+=1
                 varia=str(trackerVar)
                 varia = cv2.Tracker_create("KCF")
                 ok = varia.init(frame, bbox)
		#sets whether it should be moving forward or backward or neither
                if ((float)(yB-yA)/ratioY>1.3):
                    position=2
                    print (float)(yB-yA)/ratioY
                elif((float)(yB-yA)/ratioY<0.95):
                    position=1
                else:
                    position=0
            else:
                print "couldnt find person"
	    #logic for setting commands for drone- could be accomplished much easier with global variables
	    # but not sure if possible with current framework
                if(turn==0):
                    SendCommand(0,0,0,0)
                    rospy.sleep(0.3)
                    forward=0
                elif(turn==1):
                    SendCommand(0,0,0.3,0)
                    forward=0
                elif(turn==-1):
                    SendCommand(0,0,-0.3,0)
                    forward=0
                if(nonLocate>5):
                	SendCommand(0,0,0,0)
                	nonLocate=0
                else:
                	nonLocate+=1
            if (position==1):
                if(turn==0):
                    SendCommand(0,0.1,0,0)
                    forward=1
                elif(turn==1):
                    SendCommand(0,0.1,0.3,0)
                    forward=1
                elif(turn==-1):
                    SendCommand(0,0.1,-0.3,0)
                    forward=1
                print "forward"
                #rospy.sleep(1)
            elif(position==2):
                if(turn==0):
                    SendCommand(0,-0.1,0,0)
                    forward=2
                elif(turn==1):
                    SendCommand(0,-0.1,0.3,0)
                    forward=2
                elif(turn==-1):
                    SendCommand(0,-0.1,-0.3,0)
                    forward=2
                print "backward"
            else:
                print "nuetral"
                if(turn==0):
                    SendCommand(0,0,0,0)
                    forward=0
                elif(turn==1):
                    SendCommand(0,0,0.3,0)
                    forward=0
                elif(turn==-1):
                    SendCommand(0,0,-0.3,0)
                    forward=0
            num=0
        else:
            num+=1
	#displays current frame with bounding box drawn on
        cv2.imshow("Tracking", frame)
	#finds angle of center of bbox
        angle= findAngle((p1[0]+p2[0])/2)
        #checks angle and logic for setting drone commands
        if(angle<=5 and angle>=-5 and var>0):
            if(forward==0):
                SendCommand(0,0,0,0)
            elif(forward==1):
                SendCommand(0,0.1,0,0)
            elif(forward==-1):
                SendCommand(0,-0.1,0,0)
            turn=0
            var=0
        elif(angle>5 and var==0):
            if(forward==0):
                SendCommand(0,0,-0.3,0)
            elif(forward==1):
                SendCommand(0,0.1,-0.3,0)
            elif(forward==-1):
                SendCommand(0,-0.1,-0.3,0)

            turn=-1
            var+=1
        elif(angle<-5 and var==0):
            if(forward==0):
                SendCommand(0,0,0.3,0)
            elif(forward==1):
                SendCommand(0,0.1,0.3,0)
            elif(forward==-1):
                SendCommand(0,-0.1,0.3,0)
            turn=1
            var+=1
        #update angle
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break

def findAngle(xCoord):
	#since field of vision is around 70 degrees for AR Drone 2.0
	return round((xCoord-320)*35/320)

def initDist(frame,width,height):
    print "init"
    x=True
    par=0
    while (x):
     #creates
     hog = cv2.HOGDescriptor()
     #loads pretrained linear svm
     hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
     #resizes image for faster processing
     image = imutils.resize(frame, width=min(400, frame.shape[1]))
     orig = image.copy()
     #detects person in image
     (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        padding=(8, 8), scale=1.05)
     for (x, y, w, h) in rects:
        cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
     rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
     #applies non maxima suppression on rectangles with 65% overlap or more
     pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
     for (xA, yA, xB, yB) in pick:
        par+=1
        cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
     if(par==1):
        x=False
     else:
        msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
        frame = ros_numpy.numpify(msg)
    #returns height and width
    return (float)(xB-xA), (float)(yB-yA)

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
