#!/usr/bin/python
# import the necessary packages
from imutils.video import VideoStream
from imutils import face_utils
import datetime
#import argparse
import imutils
import time
import dlib
import cv2
import os
import rospy
import numpy as np
from std_msgs.msg import Int32,Float64,Int32MultiArray,MultiArrayLayout,MultiArrayDimension,String





import threading
import subprocess
 


pos=Int32MultiArray()
rospy.init_node('new_face_detect', anonymous=True)
rospy.set_param('/head_face/video_facial_landmarks/processid', os.getpid())
pub=rospy.Publisher('position',Int32MultiArray,queue_size = 1)
pub1=rospy.Publisher('neckposition',Int32,queue_size = 1)
# pub2 = rospy.Publisher('face_detected', String, queue_size=10)
shape_predictor='/home/asimov/IRA_V2_ws/src/new_face/src/shape_predictor_68_face_landmarks.dat'
# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(shape_predictor)

# initialize the video stream and allow the cammera sensor to warmup
print("[INFO] camera sensor warming up...")
vs = VideoStream(0).start()
time.sleep(2.0)
x1=0
y1=15
pos.data=[x1,y1]
pub.publish(pos)
frame_count=0

def callback(data):
	global process
	process = True
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


ui_trigger = rospy.Publisher('ui', String, queue_size=10)
rospy.Subscriber("ui_refresh", String, callback)




process = True



def exec_gesture(num):
   
    subprocess.call('roslaunch eva_arm_controller play3.launch', shell=True)
    
 
def exec_audio(num):
	time.sleep( 3 )
   	subprocess.call('mpg123 /home/asimov/IRA_V2_ws/src/audio/hello.mp3', shell=True)
	time.sleep( 1.5 )
	subprocess.call('mpg123 /home/asimov/IRA_V2_ws/src/audio/Press_Enter.mp3', shell=True)



switch =True
try:
	# loop over the frames from the video stream
	while True:
		if process == True :	
			frame = vs.read()
			frame = imutils.resize(frame, width=400)
			frame=cv2.flip(frame,1,0)
			#frame=imutils.rotate(frame,-90)
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			
			# detect faces in the grayscale frame
			rects = detector(gray, 0)
			if len(rects)==0:
				pub1.publish(0)
				frame_count = frame_count -1
				if frame_count < 0:
					frame_count = 0
				# print frame_count
				# pub2.publish("No")
				x1=0
				y1=15
				pos.data=[x1,y1]
				pub.publish(pos)
				# print x1,y1
			elif len(rects)==1:
				pub1.publish(1)
				frame_count = frame_count +1
				print frame_count
				# pub2.publish("Yes")
				#print rects
				for rect in rects:
					x = rect.left()
			    		y = rect.top()
			    		w = rect.right() - x
					h = rect.bottom() - y
					cv2.rectangle(frame, (x, y), (x+w,y+h), (0, 125, 255), 3)

					# recognise

					x1=(((-3*x)/20)+30)
					y1=(((-1*y)/10)+30)
					pos.data=[x1,y1]
					pub.publish(pos)
					# print x1,y1
			# loop over the face detections
			else:
				pub1.publish(3)
				# pub2.publish("Yes")
				frame_count = frame_count +1
				print rects
				cx=[]
				cy=[]
				cw=[]
				ch=[]
				for rect in rects:
					x = rect.left()
			    		y = rect.top()
			    		w = rect.right() - x
					h = rect.bottom() - y
					cv2.rectangle(frame, (x, y), (x+w,y+h), (0, 255, 127), 3)
			  		cx.append(x)
					cy.append(y)
					cw.append(w)
					ch.append(h)
				i=ch.index(max(ch))
				cv2.rectangle(frame, (cx[i], cy[i]), (cx[i]+cw[i],cy[i]+ch[i]), (255, 0,0), 3)
				x1=(((-3*cx[i])/20)+30)-(cw[i]/2)
				y1=(((-1*cy[i])/10)+30)-(ch[i]/2)
				pos.data=[x1,y1]
				pub.publish(pos)
				print x1,y1
			if frame_count >= 25:
				print frame_count
				frame_count = 0
				vs.stream.release()
				vs.stop()
				cv2.destroyAllWindows()
				switch = False
				t1 = threading.Thread(target=exec_gesture, args=(1,))
				t2 = threading.Thread(target=exec_audio, args=(1,))
				t1.start()
				t2.start()
				t1.join()
				t2.join()
				ui_trigger.publish("next")
				process = False


				# show the frame
			# cv2.imshow("Frame", frame)
			key = cv2.waitKey(1) & 0xFF
		 
			# if the `q` key was pressed, break from the loop
			if key == ord("q"):
				break
		else:
			if switch == False :
				print "*_*"
				vs = VideoStream(0).start()
				switch = True
except KeyboardInterrupt:
    print('interrupted!')
  
# do a bit of cleanup

