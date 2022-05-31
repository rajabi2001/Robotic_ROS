#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry
from HW3_scenario1.srv import ObstacleService
import numpy as np


class Detector:
    
    def __init__(self) -> None:

        rospy.init_node("get_distance" , anonymous=False)
        # rospy.on_shutdown(self.on_shutdown)

        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        rospy.Service("GetDistance" , ObstacleService , self.get_distance)


        self.myposition = (0,0)

        # building the dictionary of obstacle positions
        self.obstacles = dict()
        self.obstacles["bookshelf"] = (2.64, -1.55)
        self.obstacles["dumpster"] = (1.23, -4.57)
        self.obstacles["barrel"]	= (-2.51, -3.08)
        self.obstacles["postbox"] = (-4.47, -0.57)
        self.obstacles["brick_box"] = (-3.44, 2.75)
        self.obstacles["cabinet"] = (-0.45, 4.05)
        self.obstacles["cafe_table"]	= (1.91, 3.37)
        self.obstacles["fountain"] = (4.08, 1.14)


    def Euclidean_distance(self ,point1 ,point2):
        point1 = np.array(point1)
        point2 = np.array(point2)

        return np.linalg.norm(point1 - point2)

    def odom_callback(self, msg):

        pose = msg.pose.pose.position
        self.myposition = (pose.x, pose.y)

    def get_distance(self , req):

        obstacle_position = self.obstacles[req.obstacle_name]
        distance = self.Euclidean_distance(self.myposition ,obstacle_position)
        
        # print(distance)
        return distance






if __name__ == "__main__":

    detector = Detector()

    rospy.spin()

    
    
    

    