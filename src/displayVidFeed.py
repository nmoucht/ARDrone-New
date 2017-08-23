import numpy as np
import cv2
from ardrone_autonomy.msg import Navdata
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as imageX
import rospy
import time
import ros_numpy
rospy.init_node('takeoff', anonymous=True)

# take first frame of the video
param=True
z=0
bridge = CvBridge()
#while (param):
#ret,frame = cap.read()
#print frame
bot=0
top=0
avg=0
while(param):
    bot+=1
    start = time.time()
    msg=rospy.wait_for_message("/ardrone/image_raw", imageX)
    img = ros_numpy.numpify(msg)
    cv2.imshow('img2',img)
    #if(bot>30):
    #    param=False 


    k = cv2.waitKey(60) & 0xff
    if k == 27:
        break
cv2.destroyAllWindows()
