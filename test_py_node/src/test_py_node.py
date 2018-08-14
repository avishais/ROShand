#!/usr/bin/env python
from test_py_node.srv import ar
from std_msgs.msg import Float64MultiArray
import rospy

def talker():
    pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    array = [0.,0.]
    a = Float64MultiArray(data=array)
    # a.data = [0, 0]
    while not rospy.is_shutdown():
        # rospy.loginfo(a)
        pub.publish(a)
        array[0] += 1.1
        array[1] += 2.4
        a = Float64MultiArray(data=array)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
