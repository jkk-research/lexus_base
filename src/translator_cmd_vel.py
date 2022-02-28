#!/usr/bin/env python
# cmd_vel to /pacmod/as_rx/accel_cmd brake and steer

from distutils.command.clean import clean
import rospy
import pacmod_msgs.msg as pac_msg
from geometry_msgs.msg import Twist

class Translator:
    def __init__(self):
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.pubA = rospy.Publisher("/pacmod/as_rx/accel_cmd", pac_msg.SystemCmdFloat, queue_size=1)
        self.pubB = rospy.Publisher("/pacmod/as_rx/brake_cmd", pac_msg.SystemCmdFloat, queue_size=1)
        self.pubS = rospy.Publisher("/pacmod/as_rx/steer_cmd", pac_msg.SteerSystemCmd, queue_size=1)
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
        self.firstRun = True
        
    def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
            self.callback(self.last_published)

    def callback(self, message):
        accelCmd = pac_msg.SystemCmdFloat()
        brakeCmd = pac_msg.SystemCmdFloat()
        steerCmd = pac_msg.SteerSystemCmd()
        accelCmd.header.stamp = rospy.Time.now()
        brakeCmd.header.stamp = rospy.Time.now()
        steerCmd.header.stamp = rospy.Time.now()
        if (self.firstRun):
            accelCmd.clear_override = True
            self.firstRun = False
        else:
            accelCmd.clear_override = False
        if message.linear.x > 0.2:
            accelCmd.command = message.linear.x
            brakeCmd.command = 0.0
            accelCmd.enable = False
        elif message.linear.x < -0.1:
            accelCmd.command = 0.0
            brakeCmd.command = -1 * message.linear.x
            accelCmd.enable = True
        else:
            accelCmd.command = 0.0
            brakeCmd.command = 0.0
            accelCmd.enable = True
        steerCmd.command = message.angular.z
        steerCmd.rotation_rate = 3.3
        accelCmd.header.frame_id = "pacmod"        
        brakeCmd.enable = True
        steerCmd.enable = True
        rospy.loginfo("accel brake steer: %.1f %.1f %.1f" %(accelCmd.command, brakeCmd.command, steerCmd.command))
        self.last_published = message
        self.pubA.publish(accelCmd)
        self.pubB.publish(brakeCmd)
        self.pubS.publish(steerCmd)

if __name__ == '__main__':
    rospy.init_node("translator_cmd_vel")
    rospy.loginfo("translator_cmd_vel started")
    t = Translator()
    rospy.spin()