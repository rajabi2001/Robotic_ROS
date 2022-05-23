#!/usr/bin/env python3


import rospy
from HW1_student_filter.msg import Student 

def splite(std):
    rospy.loginfo("name:"+std.name + " "+std.last_name+" "+ "age:"+str(std.age) + " "+ ", "+std.departement)


if __name__ == '__main__':
    rospy.init_node('software')
    rospy.loginfo('The node has been strated')
    rospy.Subscriber('software',Student,callback=splite)
    rospy.spin()