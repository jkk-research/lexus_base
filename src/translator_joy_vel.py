#!/usr/bin/env python
# joy to /pacmod/as_rx/accel_cmd brake and steer

import rospy
import pacmod_msgs.msg as pac_msg
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class Translator:
    def __init__(self):
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.sub = rospy.Subscriber("/pacmod/as_tx/enabled", Bool, self.callbackEnabled)
        
        self.pubA = rospy.Publisher("pacmod/as_rx/accel_cmd", pac_msg.SystemCmdFloat, queue_size=1)
        self.pubB = rospy.Publisher("pacmod/as_rx/brake_cmd", pac_msg.SystemCmdFloat, queue_size=1)
        self.pubS = rospy.Publisher("pacmod/as_rx/steer_cmd", pac_msg.SteerSystemCmd, queue_size=1)
        #self.pubE = rospy.Publisher("pacmod/as_tx/enabled", Bool, queue_size=1)
        self.pubF = rospy.Publisher("pacmod/as_rx/enable", Bool, queue_size=1)

        self.pubTU = rospy.Publisher("pacmod/as_rx/turn_cmd", pac_msg.SystemCmdInt, queue_size=1)
        self.pubHE = rospy.Publisher("pacmod/as_rx/headlight_cmd", pac_msg.SystemCmdInt, queue_size=1)
        self.pubWI = rospy.Publisher("pacmod/as_rx/wiper_cmd", pac_msg.SystemCmdInt, queue_size=1)
        self.pubSH = rospy.Publisher("pacmod/as_rx/shift_cmd", pac_msg.SystemCmdInt, queue_size=1)
        self.pubHO = rospy.Publisher("pacmod/as_rx/horn_cmd", pac_msg.SystemCmdBool, queue_size=1)

        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./2.), self.timer_callback)
        self.joyInit = True
        self.autonomStatusChanged = True
        self.autonomStatus = True
        self.pacmodst = True
        rospy.loginfo("joy translator")
    
    def timer_callback(self, event):
        if (self.pacmodst == False):
            self.autonomStatus = False
            #rospy.logwarn("Pacmod says off")

    def callbackEnabled(self, message):
        self.pacmodst = message.data
            

    def callback(self, message):
        accelCmd = pac_msg.SystemCmdFloat()
        brakeCmd = pac_msg.SystemCmdFloat()
        steerCmd = pac_msg.SteerSystemCmd()
        TUCmd = pac_msg.SystemCmdInt()
        HECmd = pac_msg.SystemCmdInt()
        WICmd = pac_msg.SystemCmdInt()
        SHCmd = pac_msg.SystemCmdInt()
        HOCmd = pac_msg.SystemCmdBool()
        accelCmd.header.stamp = rospy.Time.now()
        brakeCmd.header.stamp = rospy.Time.now()
        steerCmd.header.stamp = rospy.Time.now()
        steerCmd.command = message.axes[0] * 20  # 6 for local 20 for laptop
        accelCmd.command = (message.axes[1] + 1) 
        brakeCmd.command = (message.axes[2] + 1) 
        if(self.autonomStatus == False):
            if(message.buttons[0]): # start A
                self.autonomStatusChanged = True
                self.autonomStatus = True
                rospy.loginfo("Autonomous mode on")
        TUCmd.command = 1
        if(message.buttons[1]): # stop B
            self.autonomStatusChanged = True
            self.autonomStatus = False
            rospy.loginfo("Autonomous mode off")
        elif(message.buttons[2]): # horn  X
            HOCmd.command = True
            rospy.loginfo("Horn")
        elif(message.axes[4] > 0): # lights 
            TUCmd.command = 2
            rospy.loginfo("Left")
        elif(message.axes[4] < 0): # lights 
            TUCmd.command = 0
            rospy.loginfo("Right")
        elif(message.axes[5] < 0): # lights 
            TUCmd.command = 3
            rospy.loginfo("Down")
        elif(message.axes[5] > 0): # lights 
            TUCmd.command = 1        
            rospy.loginfo("Up")
        if(self.autonomStatusChanged):
            accelCmd.clear_override = True
            brakeCmd.clear_override = True
            accelCmd.clear_override = True
            steerCmd.clear_override = True
            TUCmd.clear_override = True
            HECmd.clear_override = True
            WICmd.clear_override = True
            SHCmd.clear_override = True
            HOCmd.clear_override = True
        else:
            accelCmd.clear_override = False
            brakeCmd.clear_override = False
            accelCmd.clear_override = False
            steerCmd.clear_override = False
            TUCmd.clear_override = False
            HECmd.clear_override = False
            WICmd.clear_override = False
            SHCmd.clear_override = False
            HOCmd.clear_override = False
        #/pacmod/as_rx/enable.data
        status = Bool()
        status.data = self.autonomStatus
        brakeCmd.enable = self.autonomStatus
        accelCmd.enable = self.autonomStatus
        steerCmd.enable = self.autonomStatus
        TUCmd.enable = self.autonomStatus
        HECmd.enable = self.autonomStatus
        WICmd.enable = self.autonomStatus
        SHCmd.enable = self.autonomStatus
        HOCmd.enable = self.autonomStatus            
        if(self.joyInit):
            if(message.axes[1] == 0.0):
                accelCmd.command = 0.0 
            else:
                self.joyInit = False
        steerCmd.rotation_rate = 3.3
        accelCmd.header.frame_id = "pacmod"        
        ast = "----"
        if (self.autonomStatus):
            ast = "Auto  "
        else:
            ast = "Driver"
        rospy.loginfo("%s accel brake steer: %.1f %.1f %.1f" %(ast, accelCmd.command, brakeCmd.command, steerCmd.command))
        self.last_published = message
        self.pubA.publish(accelCmd)
        self.pubB.publish(brakeCmd)
        self.pubS.publish(steerCmd)
        self.pubTU.publish(TUCmd)
        self.pubHE.publish(HECmd)
        self.pubWI.publish(WICmd)
        self.pubSH.publish(SHCmd)
        self.pubHO.publish(HOCmd)
        self.pubF.publish(status)


if __name__ == '__main__':
    rospy.init_node("translator_cmd_vel")
    rospy.loginfo("translator_cmd_vel started")
    t = Translator()
    rospy.spin()