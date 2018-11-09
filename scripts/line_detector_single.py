#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import constant
import pigpio
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
N_SLICES = 5
NO_LINE = 19
BLACK = 1

def talker():
    pub = rospy.Publisher('error', Int16, queue_size=10)
    noline_pub = rospy.Publisher('noline', Int16, queue_size=10)
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=10)
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
        low_th = int(0.55*up_th)
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
    	#print len(contours)
        
        if len(contours) > 0:
            #print "Line detected"
            noline_pub.publish(1)
            pio.write(NO_LINE, 0)
            
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            #M = cv2.moments(mask)
            area = cv2.contourArea(c)
            #print area
            cx = int(M['m10']/M['m00'])
            #print cx
            error = cx  - 320
            #print "error"
            #print error
            perror = error 

            if (900 < area < 12000):
                no_line_status = 0
                no_line_counter = 0
                    
            else:
                error = perror
                #print "No line xxx"
                no_line_counter = no_line_counter + 1
                #print no_line_counter
                #noline_pub.publish(0)
                if no_line_counter > 20:
                    #p_state = STATE.value
                    #STATE.value = state_stop

                    no_line_status = 1
                    pio.write(NO_LINE, 1)
                    print "No line"
                    noline_pub.publish(0)

                

    	else:
            error = perror
            no_line_counter = no_line_counter + 1
            if no_line_counter > 20:
                #p_state = STATE.value
                #STATE.value = state_stop
                
                no_line_status = 1
                pio.write(NO_LINE, 1)
                print "No line"
                noline_pub.publish(0)



        #c = max(contours, key=cv2.contourArea)
        cv2.drawContours(images[selectedPoint], contours, -1, (0,255,0), 3)

        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(error)
        pub.publish(error) 


        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', images[selectedPoint])[1]).tostring()
        # Publish new image
        image_pub.publish(msg)
        rate.sleep()



if __name__ == '__main__':
    cam = cv2.VideoCapture(0)
    cam.set(5 , 30) 
    pio = pigpio.pi()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
