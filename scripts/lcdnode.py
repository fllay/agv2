#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16
import I2C_LCD_driver
import time
import datetime

import pylcdlib
#import subprocess
 
#lcd = pylcdlib.lcd(0x27,1) #I2C apparaat op adres 0x27.
#lcd = pylcdlib.lcd(0x27,1) #I2C apparaat op adres 0x27.

def callback(data):
    global lcd_counter
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%M:%S')
    #strs = st + " : State = " + str(data.data)
    #ts = str(data.stamp)
    #mylcd.lcd_display_string(ts, 2)
    strs = "State = " + str(data.data)
    #mylcd.lcd_display_string(strs, 3)

    #time.sleep(0.05)
    lcd_counter = lcd_counter + 1
    #print lcd_counter
    if(lcd_counter%10 == 0):
        
        lcd_counter = 0
        try:
            #lcd.lcd_write(0x0C) #Cursor uitschakelen.
            #time.sleep(0.05) 
            #lcd.lcd_write(0x01) #Scherm leegmaken.
            #time.sleep(0.05) 
            #lcd.lcd_puts(strs, 1) #Tekst voor LCD display lijn 1. 
            #lcd.lcd_backlight(1) #Achterverlichting aanzetten.
            mylcd.lcd_display_string(strs, 1)
        except IOError:
            #subprocess.call(['i2cdetect', '-y', '1'])
            #optionele vlag als signaal om de gegevens eventueel nog een
            #keer naar het display te sturen.
            flag = 1

    #lcd.lcd_puts(st, 2) #Tekst voor LCD display lijn 2.
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("state", Int16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lcd_counter = 0
    mylcd = I2C_LCD_driver.lcd()
    listener()