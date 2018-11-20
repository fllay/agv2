#!/usr/bin/env python

import numpy as np
import cv2
import ivport
import rospy
import IIC
import constant
import pigpio
from multiprocessing import Value
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
N_SLICES = 5
NO_LINE = 19
BLACK = 0
current_dir = 2
#state = constant.state_run_forward

def camCallBack(data):
    global current_dir
    rospy.loginfo(rospy.get_caller_id() + 'CAM # %d', data.data)  
    if(data.data != current_dir):
        iv.camera_change(data.data)
        current_dir = data.data
def StateCallback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard state %s', data.data)    
    STATE.value = data.data
    #rospy.loginfo(rospy.get_caller_id() + 'I heard state %s', STATE.value)

def talker():
    pub = rospy.Publisher('error', Int16, queue_size=10)
    
    pub_inline = rospy.Publisher('inline', Int16, queue_size=10)
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=10)
    rospy.Subscriber('/cam_dir', Int16, camCallBack)
    rospy.Subscriber('/state', Int16, StateCallback)
    rospy.init_node('line_detector', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    images = []
    for i in range(N_SLICES):
        images.append(np.zeros((100,640,3), np.uint8))	
    perror = 0
    no_line_counter = 0
    no_line_status = 0
    while not rospy.is_shutdown():
        success, image0 = cam.read()
        color_idx = 0
    	

    
        selectedPoint = 3
        #image0 = vs.read()
		# grab the frame from the stream and resize it to have a maximum
        # width of 400 pixels
        if selectedPoint == 0:
            images[0] = image0[0:99, 0:639]
        elif selectedPoint == 1:
            images[1] = image0[80:179, 0:639]
        elif selectedPoint == 2:
            images[2] = image0[180:279, 0:639]
        elif selectedPoint == 3:
            images[3] = image0[280:379, 0:639]
        elif selectedPoint == 4:
            images[4] = image0[380:479, 0:639]

        #gray = cv2.bitwise_not(cv2.cvtColor(images[selectedPoint], cv2.COLOR_BGR2GRAY))
    	#gray = cv2.cvtColor(images[selectedPoint], cv2.COLOR_BGR2GRAY)

        img_yuv = cv2.cvtColor(images[selectedPoint], cv2.COLOR_BGR2YUV)
        y, u, v = cv2.split(img_yuv)
        if(BLACK == 1):
            y = cv2.bitwise_not(y)
        up_th = int(np.amax(y))
        low_th = int(0.60*up_th)
        #print up_th
        #print low_th
        # Gaussian blur
        img = cv2.GaussianBlur(y, (5, 5), 0)
        #Color thresholdingret, thresh = cv2.threshold(blur, 100, 200, cv2.THRESH_BINARY_INV)
        
        ret,thresh = cv2.threshold(img,low_th,up_th,cv2.THRESH_BINARY)
        #blur = cv2.GaussianBlur(gray, (5, 5), 0)
    	#Color thresholding
    	#ret, thresh = cv2.threshold(blur, 100, 200, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    	#thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 311,60)
        thresh = cv2.erode(thresh, None, iterations=4)
        thresh = cv2.dilate(thresh, None, iterations=4)


        hsv = cv2.cvtColor(images[selectedPoint], cv2.COLOR_BGR2HSV)

        # define range of white color in HSV
        # change it according to your need !
        #lower_white = np.array([0,0,0], dtype=np.uint8)
        #upper_white = np.array([0,0,255], dtype=np.uint8)

        sensitivity = 60
        #lower_white = np.array([0,0,255-sensitivity])
        #upper_white = np.array([255,sensitivity,255])

        lower_white = np.uint8([23,41,133])
        upper_white = np.uint8([40,150,255])
        
        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)
        # Bitwise-AND mask and original image
        #thresh = cv2.bitwise_and(gray,gray, mask= mask)
        im2, contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

        # Find the biggest contour (if detected)
    	#print "damn"
    	print len(contours)
        
        if len(contours) > 0:
            #print "Line detected"	
            pio.write(NO_LINE, 0)
           
            if STATE.value == constant.state_turn_left:
                largest_contours = sorted(contours, key=cv2.contourArea)[-3:]
                print "Number of three largest "	
                print len(largest_contours)
                errora = []
                for c in largest_contours:
                    M = cv2.moments(c)
                    cx = int(M['m10']/M['m00'])
                    errora.append(cx  - 320)
                print(errora)
                print(min(errora))
                error = min(errora)
            elif STATE.value == constant.state_turn_right:
                largest_contours = sorted(contours, key=cv2.contourArea)[-3:]
                print "Number of three largest "	
                print len(largest_contours)
                errora = []
                for c in largest_contours:
                    M = cv2.moments(c)
                    cx = int(M['m10']/M['m00'])
                    errora.append(cx  - 320)
                print(errora)
                print(max(errora))
                error = max(errora)
            else : 
                c = max(contours, key=cv2.contourArea)
                approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)
                #rospy.loginfo('number of coners %d', len(approx))  
                M = cv2.moments(c)
                area = cv2.contourArea(c)
                print "area = "
                print(area)
            
                #M = cv2.moments(mask)

                cx = int(M['m10']/M['m00'])
                #print cx
                if len(contours) > 1:
                    error = perror
                elif len(contours) == 1:   
                    error = cx  - 320
                
                #print "error"
                #print error
                perror = error

                # if (len(approx) < 5):
                #     no_line_status = 0
                #     no_line_counter = 0
                # else:
                #     error = perror
                #     #print "No line xxx"
                #     no_line_counter = no_line_counter + 1
                #     #print no_line_counter
                #     if no_line_counter > 20:
                #         #p_state = STATE.value
                #         #STATE.value = state_stop
            
                #         no_line_status = 1
                #         pio.write(NO_LINE, 1)
                #         #print "No line"
                #         rospy.loginfo("No line")

                if (3000 < area < 12000):
                    no_line_status = 0
                    no_line_counter = 0
                else:
                    error = perror
                    #print "No line xxx"
                    no_line_counter = no_line_counter + 1
                    #print no_line_counter
                    if no_line_counter > 20:
                        #p_state = STATE.value
                        #STATE.value = state_stop
                
                        no_line_status = 1
                        pio.write(NO_LINE, 1)
                        print "No line"

        else:
            error = perror
            no_line_counter = no_line_counter + 1
            if no_line_counter > 20:
                #p_state = STATE.value
                #STATE.value = state_stop
                
                no_line_status = 1
                pio.write(NO_LINE, 1)
                print "No line"


        #c = max(contours, key=cv2.contourArea)
        cv2.drawContours(images[selectedPoint], contours, -1, (0,255,0), 3)

        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(error)
        pub.publish(error)
        if -50 < error < 50:
            pub_inline.publish(1)
            #print("inline")
        else:
            pub_inline.publish(0)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', images[selectedPoint])[1]).tostring()
        # Publish new image
        image_pub.publish(msg)
        rate.sleep()



if __name__ == '__main__':
    iviic = IIC.IIC(addr=(0x70), bus_enable =(0x01))
    iviic.write_control_register((0x01))                # default is channel 1

    iv = ivport.IVPort(ivport.TYPE_DUAL2)
    iv.camera_change(2)
    current_dir = 2
    pio = pigpio.pi()
    cam = cv2.VideoCapture(0)
    cam.set(5 , 30) 
    STATE = Value('i', constant.state_stop)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
