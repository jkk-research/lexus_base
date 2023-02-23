#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
try:
    import pandas as pd
except:
    print("pip install pandas")
import rospkg
import numpy as np




def talker():
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('cmd_vel_from_file', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg_twist = Twist()
    csv_file = rospy.get_param('~csv')
    rospack = rospkg.RosPack()
    path = rospack.get_path('lexus_base')
    rospy.loginfo('csv file is: %s/etc/%s' % (path, csv_file))
    array = pd.read_csv(path + '/etc/' + csv_file, header = None).to_numpy()
    #rospy.loginfo(array[:,0])
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        current_time = (rospy.Time.now() - start_time).to_sec()
        itemindex = np.where(array[:,0] >= current_time)
        try:
            msg_twist.linear.x = array[itemindex[0][0],1]
            rospy.loginfo("%6.2fs %4.2fm/s", current_time, array[(itemindex)[0][0],1])
        except:
            msg_twist.linear.x = 0.0
        cmd_pub.publish(msg_twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass