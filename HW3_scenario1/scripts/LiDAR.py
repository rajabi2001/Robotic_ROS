#!/usr/bin/env python3


from traceback import print_tb
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from HW3_scenario1.msg import Obstacle
from math import radians
import numpy as np


class Sensor:
    
    def __init__(self) -> None:

        rospy.init_node("LiDAR_sensor" , anonymous=False)

        self.obstacle_subscriber = rospy.Subscriber("/ClosestObstacle" , Obstacle , callback=self.obstacle_callback)
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher("/cmd_vel" , Twist , queue_size=10)


        self.closest_distance = 999
        self.closest_obstacle = "nothing"




    def obstacle_callback(self ,msg):

        self.closest_distance = msg.distance
        self.closest_obstacle = msg.obstacle_name
        

    # def laser_callback(self ,msg):

    #     pass
    def find_min_angle(self , laser_msg):

        min_distance = 999
        myangle = 0

        for i in range(360):

            if laser_msg.ranges[i] < min_distance:
                myangle = i
                min_distance = laser_msg.ranges[i]
        
        return myangle
            

    def run(self):

        while not rospy.is_shutdown():

            if self.closest_distance < 1.5:

                # print(f"obstacle:{self.closest_obstacle} and distance:{self.closest_distance}")
                self.cmd_publisher.publish(Twist())

                laser_msg = rospy.wait_for_message("/scan" , LaserScan)
                myangle = self.find_min_angle(laser_msg)

                print(myangle)






if __name__ == "__main__":

    sensor = Sensor()

    sensor.run()