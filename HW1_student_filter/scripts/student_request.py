#!/usr/bin/env python3

import rospy
from HW1_student_filter.msg import Student 
from random_student import randStudent


if __name__ == '__main__':
    rospy.init_node('student_request')
    rospy.loginfo('The node has been strated')
    pub = rospy.Publisher('std_request',Student,queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        std = randStudent()
        pub.publish(std)
        # rospy.loginfo(std.name)
        rate.sleep()
