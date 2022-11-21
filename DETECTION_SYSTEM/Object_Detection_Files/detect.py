import cv2
import numpy as np
import time 
from PIL import Image
from picamera.array import PiRGBArray
from picamera import PiCamera

thres = 0.6 # Threshold to detect object
nms_threshold = 0.2
 
classNames= []
classFile = 'coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n') 


configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt' #mAp 22% 5 Fps
weightsPath = 'frozen_inference_graph.pb'
 
net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(128,128)
#net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

from picamera.array import PiRGBArray
from picamera import PiCamera

# initialize the camera and grab a reference to the raw camera capture
prev_frame_time = 0 
# used to record the time at which we processed current frame
new_frame_time = 0
camera = PiCamera()
#camera.resolution = (640, 480)
camera.resolution = (320, 320)
#camera.resolution = (160, 120)
camera.framerate = 15
#rawCapture = PiRGBArray(camera, size=(640, 480))
rawCapture = PiRGBArray(camera, size=(320, 320))
#rawCapture = PiRGBArray(camera, size=(160, 120))

# allow the camera to warmup
time.sleep(5)
# capture frames from the camera
for fram in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	frame = fram.array
	orig = frame.copy()
	classIds, confs, bbox = net.detect(frame,confThreshold=thres)   
	indices= cv2.dnn.NMSBoxes(bbox,confs, thres,  nms_threshold)
	
	# for classId, confidence, box in zip (classIds, confs, bbox):
	# 	cv2.rectangle(orig, box, color=(0, 255, 0), thickness=1)
	# 	cv2.putText(orig, classNames[classId-1], (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255, 0))
	# 	cv2.putText(orig, str("{0:.1f}".format(confidence*100)), (box[0]+15, box[1]+50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255, 0))

	# bbox = list(bbox)
	# confs = list(np.array(confs).reshape(1,-1)[0])
	# confs = list(map(float,confs))
    #print(type(confs[0]))
    #print(confs)
 
	indices = cv2.dnn.NMSBoxes(bbox,confs,thres,nms_threshold)
    #print(indices)
 
	for i in indices:
		#i = i[0]
		box = bbox[i]
		x,y,w,h = box[0],box[1],box[2],box[3]
		# cv2.rectangle(orig, (x,y),(x+w,h+y), color=(0, 255, 0), thickness=1)
		# cv2.putText(orig,classNames[classIds[i]-1].upper(),(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,255,0),1)
		# cv2.putText(orig, str("{0:.1f}".format(confs[i]*100)), (box[0]+15, box[1]+50), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,255, 0))
		
		if classNames[classIds[i]-1]== "person" or classNames[classIds[i]-1]== "boat":
			col=(255, 0, 0)
			cv2.rectangle(orig, (x,y),(x+w,h+y), color=col, thickness=1)
			cv2.putText(orig,classNames[classIds[i]-1].upper(),(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_COMPLEX,0.5,col,1)
			cv2.putText(orig, str("{0:.1f}".format(confs[i]*100)), (box[0]+15, box[1]+50), cv2.FONT_HERSHEY_COMPLEX, 0.5, col)
	
	#font = cv2.FONT_HERSHEY_SIMPLEX
	new_frame_time = time.time() 
	fps = 1/(new_frame_time-prev_frame_time)
	prev_frame_time = new_frame_time 
	fps = float(fps) 
	fps = str("{0:.1f}".format(fps)+" fps")
	cv2.putText(orig, fps , (10, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
	print(fps)
	
	cv2.imshow('video2', orig)
	rawCapture.truncate(0)
	if cv2.waitKey(1) & 0xFF == ord('q'): # Se esperan 30ms para el cierre de la ventana o hasta que el usuario precione la tecla q
		break


cv2.destroyAllWindows()