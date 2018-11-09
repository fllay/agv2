#!/usr/bin/env python 
import rospy
import constant
import pigpio
from time import sleep
from std_msgs.msg import Int16
from multiprocessing import Value
from dynamic_reconfigure.server import Server
from agv2.cfg import agv2Config
from agv2.srv import *

MAX_SPEED = 200000
SLOW_SPEED = 150000
SLOW_STRAIGHT_SPEED = 120000
VERY_SLOW_SPEED = 55000
LEFT_SPEED = 100000
RIGHT_SPEED = 100000
DIR_PIN_LEFT = 19
DIR_PIN_RIGHT = 20

CLEAR_ROTATION_BIT = 26
MAX_ROTATION_COUNT = 90
#CLEAR_ROTATION_BIT = 0

#CAM_DIR = 1

class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=1000, Integrator_min=-1000):

            self.Kp=P
            self.Ki=I
            self.Kd=D
            self.Derivator=Derivator
            self.Integrator=Integrator
            self.Integrator_max=Integrator_max
            self.Integrator_min=Integrator_min

            self.set_point=0.0
            self.error=0.0

	def update(self,current_value):
            """
            Calculate PID output value for given reference input and feedback
            """

            self.error = self.set_point - current_value

            self.P_value = self.Kp * self.error
            self.D_value = self.Kd * ( self.error - self.Derivator)
            self.Derivator = self.error

            self.Integrator = self.Integrator + self.error

            if self.Integrator > self.Integrator_max:
                self.Integrator = self.Integrator_max
            elif self.Integrator < self.Integrator_min:
                self.Integrator = self.Integrator_min

            self.I_value = self.Integrator * self.Ki

            PID = self.P_value + self.I_value + self.D_value

            return PID

	def setPoint(self,set_point):
            """
            Initilize the setpoint of PID
            """
            self.set_point = set_point
            self.Integrator=0
            self.Derivator=0

	def setIntegrator(self, Integrator):
            self.Integrator = Integrator

	def setDerivator(self, Derivator):
            self.Derivator = Derivator

	def setKp(self,P):
            self.Kp=P
                
	def setKi(self,I):
            self.Ki=I

	def setKd(self,D):
            self.Kd=D

	def getPoint(self):
            return self.set_point
            
	def getError(self):
            return self.error
            
	def getIntegrator(self):
            return self.Integrator
            
	def getDerivator(self):
            return self.Derivator

def CamCallback(data):
#        rospy.loginfo(rospy.get_caller_id() + 'IDrive CAM  %d', data.data)   
        if data.data == 1:
                CAM_DIR.value = 1
#                rospy.loginfo(rospy.get_caller_id() + 'Cam callbcak Drive CAM  %d', CAM_DIR)
        elif data.data == 2:
                CAM_DIR.value = 2
#                rospy.loginfo(rospy.get_caller_id() + 'CAM callback Drive CAM  %d', CAM_DIR)
#        rospy.loginfo(rospy.get_caller_id() + 'IDrive CAM_DIR  %d', CAM_DIR)        

def StateCallback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard state %s', data.data)    
    STATE.value = data.data
    #rospy.loginfo(rospy.get_caller_id() + 'I heard state %s', STATE.value)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    error = data.data
    t_counter = 0
    #rotationCount = 0
    #rospy.loginfo(rospy.get_caller_id() + 'Now state =  %s', STATE.value)

    if(STATE.value == constant.state_run_forward):
        pid = pidc.update(error)
    elif(STATE.value == constant.state_run_backward):
        pid = pidc.update(error)
    elif(STATE.value == constant.state_run_slow):
        pid = pidSlow.update(error)		
    elif(STATE.value == constant.state_run_very_slow):
        pid = pidVerySlow.update(error)
    elif(STATE.value == constant.state_turn_left):
        pid = pidLeft.update(error)	
    elif(STATE.value == constant.state_turn_right):
        pid = pidRight.update(error)
    elif(STATE.value == constant.state_run_slow_straight):
        pid = pidSlowStraight.update(error)		

        #print "ddddddddddddd"
        #right motor
    #pio.write(CLEAR_ROTATION_BIT,0) 
    if(STATE.value == constant.state_stop):
        #rospy.loginfo("STOP") 
        pio.hardware_PWM(13, 800, 0)
        #left motor
        pio.hardware_PWM(12, 800, 0)
        daccelCount.value = 0
        b_accelCount.value = 0
        f_accelCount.value = 0
        vdaccelCount.value = 0
    elif(STATE.value == constant.state_run_forward):
        rotationCount.value = 0
        daccelCount.value = 0
        b_accelCount.value = 0
        vdaccelCount.value = 0
        print "running forward"
        #print accelCount.value
        
        if f_accelCount.value < 25:
                speed = maxspeed_param.value*0.40
                print "25%"
        elif 25 <= f_accelCount.value  < 50:
                speed = maxspeed_param.value*0.50
                print "50%"
        elif 50 <= f_accelCount.value  <= 75:
                speed = maxspeed_param.value*0.75
                print "75%"
        elif f_accelCount.value > 75:
                f_accelCount.value = 100
                speed = maxspeed_param.value
                print "100%"
			
        #accelCount.value = accelCount.value + 1

        f_accelCount.value = f_accelCount.value + 1
        lefts = speed + pid
        #print(speed)
        if(lefts < 0):
                lefts = 0
        elif(lefts > 999999):
                lefts = 999999
        rights = speed - pid
        #print(rights)
        if(rights < 0):
                rights = 0
        elif(rights > 999999):
                rights = 999999
        #right motor
        pio.hardware_PWM(13, 800, rights)
        #left motor
        pio.hardware_PWM(12, 800, lefts)
        t_counter = 0
        pio.write(CLEAR_ROTATION_BIT,0)
        
                        
    elif(STATE.value == constant.state_run_backward):
        
        rotationCount.value = 0        
        f_accelCount.value = 0
        daccelCount.value = 0
        vdaccelCount.value = 0
        print "running backwaerd"
        #print accelCount.value
        
        if b_accelCount.value < 25:
                speed = maxspeed_param.value*0.40
                print "25%"
        elif 25 <= b_accelCount.value < 50:
                speed = maxspeed_param.value*0.50
                print "50%"
        elif 50 <= b_accelCount.value <= 75:
                speed = maxspeed_param.value*0.75
                print "75%"
        elif b_accelCount.value > 75:
                b_accelCount.value = 100
                speed = maxspeed_param.value
                print "100%"
                
        b_accelCount.value = b_accelCount.value + 1

        #accelCount.value = accelCount.value + 1
        lefts = speed - pid
        #print(speed)
        if(lefts < 0):
                lefts = 0
        elif(lefts > 999999):
                lefts = 999999
        rights = speed + pid
        #print(rights)
        if(rights < 0):
                rights = 0
        elif(rights > 999999):
                rights = 999999
        #right motor
        pio.hardware_PWM(13, 800, rights)
        #left motor
        pio.hardware_PWM(12, 800, lefts)
        t_counter = 0
        pio.write(CLEAR_ROTATION_BIT,0)
        
                        
    elif(STATE.value == constant.state_run_slow):
        b_accelCount.value = 0
        f_accelCount.value = 0
        vdaccelCount.value = 0
        
        """if daccelCount.value < 20:
            speed = maxspeed_param.value*0.75
            print "25%"
            daccelCount.value = daccelCount.value + 1
        elif 20 <= daccelCount.value < 40:
            speed = maxspeed_param.value*0.50
            daccelCount.value = daccelCount.value + 1
            print "50%"
        elif 40 <= daccelCount.value <= 60:
            speed = maxspeed_param.value*0.25
            daccelCount.value = daccelCount.value + 1
            print "75%"
        elif daccelCount.value > 60:
            speed = slowspeed_param.value
            print "100%"
	"""		
        #print "Deccel = ",
        #print "RUN SLOW"
        speed = slowspeed_param.value
        lefts = speed + pid
        #print(speed)
        if(lefts < 0):
            lefts = 0
        elif(lefts > 999999):
            lefts = 999999
        rights = speed - pid
        #print(rights)
        if(rights < 0):
                rights = 0
        elif(rights > 999999):
                rights = 999999
        rospy.loginfo(rospy.get_caller_id() + 'Very slow  Drive Drive CAM_DIR  %d', CAM_DIR.value)
        if CAM_DIR.value == 1:
                #print "CAM1""
                #right motor
                pio.hardware_PWM(13, 800, rights)
                #left motor
                pio.hardware_PWM(12, 800, lefts)
        elif CAM_DIR.value == 2:
                #print "CAM2"
                #right motor
                pio.hardware_PWM(12, 800, rights)
                #left motor
                pio.hardware_PWM(13, 800, lefts)

        t_counter = 0
        pio.write(CLEAR_ROTATION_BIT,0)
    elif(STATE.value == constant.state_run_very_slow):
        b_accelCount.value = 0
        f_accelCount.value = 0
        daccelCount.value = 0
        """if vdaccelCount.value < 20:
            speed = maxspeed_param.value*0.02
            print "25%"
            vdaccelCount.value = vdaccelCount.value + 1
        elif 20 <= vdaccelCount.value < 40:
            speed = maxspeed_param.value*0.03
            vdaccelCount.value = vdaccelCount.value + 1
            print "50%"
        elif 40 <= vdaccelCount.value <= 60:
            speed = maxspeed_param.value*0.05
            vdaccelCount.value = vdaccelCount.value + 1
            print "75%"
        elif vdaccelCount.value > 60:
            speed = VERY_SLOW_SPEED
            print "100%"
        """
        speed = VERY_SLOW_SPEED
        lefts = speed + pid
        #print(speed)
        if(lefts < 0):
            lefts = 0
        elif(lefts > 999999):
            lefts = 999999
        rights = speed - pid
        #print(rights)
        if(rights < 0):
            rights = 0
        elif(rights > 999999):
            rights = 999999
        if CAM_DIR.value == 1:
                #print "CAM1"
                #right motor
                pio.hardware_PWM(13, 800, rights)
                #left motor
                pio.hardware_PWM(12, 800, lefts)
        elif CAM_DIR.value == 2:
                #print "CAM2"
                #right motor
                pio.hardware_PWM(12, 800, rights)
                #left motor
                pio.hardware_PWM(13, 800, lefts)
            
        pio.write(CLEAR_ROTATION_BIT,0)
    elif(STATE.value == constant.state_turn_left):
        b_accelCount.value = 0
        f_accelCount.value = 0
        daccelCount.value = 0
        vdaccelCount.value = 0
        #pid = pidSlow.update(error)	
        t_counter = t_counter + 1
        if t_counter < 50:
            ll_speed = -45000		
        else:
            ll_speed = pid
            #print "turn_keft"
        speed = LEFT_SPEED
        lefts = speed + ll_speed
        #print(speed)
        if(lefts < 0):
            lefts = 0
        elif(lefts > 999999):
            lefts = 999999
        rights = speed - ll_speed
        #print(rights)
        if(rights < 0):
            rights = 0
        elif(rights > 999999):
            rights = 999999
        #right motor
        pio.hardware_PWM(13, 800, rights)
        #left motor
        pio.hardware_PWM(12, 800, lefts)
        pio.write(CLEAR_ROTATION_BIT,0)
    elif(STATE.value == constant.state_turn_right):
        speed = RIGHT_SPEED
        f_accelCount.value = 0
        b_accelCount.value = 0
        daccelCount.value = 0
        vdaccelCount.value = 0
        t_counter = t_counter + 1
        #pid = pidSlow.update(error)	
        if t_counter < 50:
            ll_speed = 45000
        else:
            ll_speed = pid
            #print "turn_right = ",
            #print t_counter,
	    
        lefts = speed + ll_speed
        #print(speed)
        if(lefts < 0):
            lefts = 0
        elif(lefts > 999999):
            lefts = 999999
        rights = speed - ll_speed
        #print(rights)
        if(rights < 0):
            rights = 0
        elif(rights > 999999):
            rights = 999999
        #right motor
        pio.hardware_PWM(13, 800, rights)
        #left motor
        pio.hardware_PWM(12, 800, lefts)
         #print "State = ",
        #print STATE.value,
        pio.write(CLEAR_ROTATION_BIT,0)
    elif(STATE.value == constant.state_run_slow_straight):
        vdaccelCount.value = 0
        f_accelCount.value = 0
        b_accelCount.value = 0
        print "Deccel = ",
        print daccelCount.value
        speed = SLOW_STRAIGHT_SPEED
        lefts = speed + pid
        #print(speed)
        if(lefts < 0):
        	lefts = 0
        elif(lefts > 999999):
        	lefts = 999999
        rights = speed - pid
        #print(rights)
        if(rights < 0):
            rights = 0
        elif(rights > 999999):
            rights = 999999
            
        if CAM_DIR.value == 1:
                #print "CAM1"
                #right motor
                pio.hardware_PWM(13, 800, rights)
                #left motor
                pio.hardware_PWM(12, 800, lefts)
        elif CAM_DIR.value == 2:
                #print "CAM2"
                #right motor
                pio.hardware_PWM(12, 800, rights)
                #left motor
                pio.hardware_PWM(13, 800, lefts)
        t_counter = 0

    elif(STATE.value == constant.state_rotate_ccw):

        print "CW !!!!!!!!!!!!!!!!!!!!!!!"
        rospy.loginfo("CCW!!!!!!")
        pio.hardware_PWM(13, 800, 0)
        #left motor
        pio.hardware_PWM(12, 800, 0)
        #if carType.Value == 'Car_test':
        """pio.write(DIR_PIN_LEFT,0)
    	pio.write(DIR_PIN_RIGHT,0)"""
        rotatingDir.value = 0
        isRotating.value = 1
        stopSignal = 0
        #for x in range(70):
        while rotationCount.value < MAX_ROTATION_COUNT:
                if STATE.value == constant.state_stop:
                        rotationCount.value = rotationCount.value - 10
                        stopSignal = 1
                        break
                elif STATE.value != constant.state_rotate_ccw:
                        break
                else:
                        pio.hardware_PWM(13, 800, 40000)
                        #left motor
                        pio.hardware_PWM(12, 800, 40000)
                sleep(0.1)
                print(rotationCount.value)
                rotationCount.value  += 1
        if rotationCount.value == MAX_ROTATION_COUNT:
                pio.write(CLEAR_ROTATION_BIT,1)
                print("Clear")
                sleep(2)
        if stopSignal == 0:
                rotationCount.value = 0
                isRotating.value = 0
        elif stopSignal == 1:
                isRotating.value = 1
        pio.hardware_PWM(13, 800, 0)
        #left motor
        pio.hardware_PWM(12, 800, 0)
        
                
        
        STATE.value = constant.state_stop
        
    elif(STATE.value == constant.state_rotate_cw):
        print "CW !!!!!!!!!!!!!!!!!!!!!!!"
        rospy.loginfo("CW!!!!!!")
        pio.hardware_PWM(13, 800, 0)
        #left motor
        pio.hardware_PWM(12, 800, 0)
        #if carType.Value == 'Car_test':
        """pio.write(DIR_PIN_LEFT,1)
        pio.write(DIR_PIN_RIGHT,1)"""
        rotatingDir.value = 1
        isRotating.value = 1
        stopSignal = 0
        #for x in range(70):
        
        while rotationCount.value < MAX_ROTATION_COUNT:
                if STATE.value == constant.state_stop:
                        stopSignal = 1
                        rotationCount.value = rotationCount.value - 10
                        break
                elif STATE.value != constant.state_rotate_cw:
                        break
                else:
                        stopSignal = 0
                        pio.hardware_PWM(13, 800, 40000)
                        #left motor
                        pio.hardware_PWM(12, 800, 40000)
                sleep(0.1)
                print(rotationCount.value)
                rotationCount.value += 1
        if rotationCount.value == MAX_ROTATION_COUNT:
                pio.write(CLEAR_ROTATION_BIT,1)
                print("Clear ")
                sleep(2)
        if stopSignal == 0:
                isRotating.value = 0
                rotationCount.value = 0
        elif stopSignal == 1:
                isRotating.value = 1
        pio.hardware_PWM(13, 800, 0)
        #left motor
        pio.hardware_PWM(12, 800, 0)
                
                                
        STATE.value = constant.state_stop
        
    else:
        pio.hardware_PWM(13, 800, 0)
        #left motor
        pio.hardware_PWM(12, 800, 0)

def parameterCallback(config, level):
        rospy.loginfo("MAX_ROTATION_COUNT = %d",  MAX_ROTATION_COUNT)
        #rospy.loginfo("maxspeed_param.value = %d",  maxspeed_param.value)
        rospy.loginfo("MAX_SPEED_PARAM = %d",  config["maximun_speed"])
        if "maximun_speed" in config:
            maxspeed_param.value = config["maximun_speed"]
        if "slow_speed" in config:
            slowspeed_param.value = config["slow_speed"]
        #slowspeed_param.value
        rospy.loginfo("""Reconfigure Request: {maximun_speed}""".format(**config))
        return config

def handle_get_param(req):
    print("Returning " + req.words)
    if(req.words == "maxspeed"):
        valll = maxspeed_param.value
    elif(req.words == "slowspeed"):
        valll = slowspeed_param.value
    return valll

def listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously
        
        rospy.init_node('drive_motor', anonymous=True)

        rospy.Subscriber('state', Int16, StateCallback)
        rospy.Subscriber('error', Int16, callback)
        rospy.Subscriber('cam_dir', Int16, CamCallback)    
        #s = rospy.Service('get_maxspeed', AddTwoInts, handle_add_two_ints)
        # spin() simply keeps python from exiting until this node is stopped
        srv = Server(agv2Config, parameterCallback)
        s = rospy.Service('get_param', getparamm, handle_get_param)
        rospy.spin()

if __name__ == '__main__':

    #PID 300000 450
    pidSlow=PID(450.0,0.00001,1000.0)
    pidSlow.setPoint(0)

    #PID 710000
    #pidc=PID(500.0,0.000001,8900.0)  #150000

    #pidc=PID(85, 0.000001, 1750) #200000
    pidc=PID(300, 0.000001, 1000) #200000
    #pidc=PID(1000, 15, 35000)
    
    pidc.setPoint(0)
    
    pidLeft=PID(85.0,0.000001,3500.0)
    pidLeft.setPoint(0)

    pidRight=PID(85.0,0.000001,3500.0)
    pidRight.setPoint(0)

    #slow straight
    pidSlowStraight = PID(150.0,0.00001,2500.0)
    pidSlowStraight.setPoint(0)


    #PID very slow  
    pidVerySlow = PID(100.0,0.000001,1750.0)
    pidVerySlow.setPoint(0)

    #MAX_ROTATION_COUNT = rospy.get_param('/drive_motor/rotationVal')
    #MAX_SPEED = rospy.get_param('/drive_motor/maxSpeed')
    STATE = Value('i', constant.state_stop)
    CAM_DIR = Value('i', 1)
    f_accelCount = Value('i', 0)
    b_accelCount = Value('i', 0)    
    daccelCount = Value('i', 0)
    isRotating = Value('i', 0)
    vdaccelCount = Value('i', 0)
    rotatingDir = Value('i',0)
    rotationCount = Value('i',0)
    maxspeed_param = Value('i',MAX_SPEED)
    slowspeed_param = Value('i',SLOW_SPEED)
    carType = Value('i', 0)  #small car = 0 large car = 1
    pio = pigpio.pi()
    pio.set_mode(CLEAR_ROTATION_BIT, pigpio.OUTPUT)

    
    zeroHasDetected = 0
    turn_counter = 0
    no_line_counter = 0
    
    t_counter = 0 
    check_io = 1
    p_state = constant.state_run_forward
    no_line_status = 0

    listener()
