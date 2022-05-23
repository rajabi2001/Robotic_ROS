#!/usr/bin/env python3


import rospy
from HW1_student_filter.msg import Student 

def splite(std):
    # rospy.loginfo(std.name + " "+ std.departement)
    pubs = rospy.Publisher('software',Student,queue_size=10)
    pubh = rospy.Publisher('hardware',Student,queue_size=10)


    if std.departement == "Hardware":
        pubh.publish(std)     
    else:
        pubs.publish(std)

    rospy.loginfo(std.name + " "+ std.departement)


if __name__ == '__main__':
    rospy.init_node('splitter')
    rospy.loginfo('The node has been strated')
    rospy.Subscriber('std_request',Student,callback=splite)
    rospy.spin()