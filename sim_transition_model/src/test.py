#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, Empty, EmptyResponse

def Test(msg):
    print('In callback.')

    return EmptyResponse

if __name__ == '__main__':

    rospy.init_node('test', anonymous=True)

    rospy.Service('/test', Empty, Test)
    rospy.spin()