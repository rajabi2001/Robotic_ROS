#!/usr/bin/env python3


from pickle import TRUE
from time import sleep
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
        self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher("/cmd_vel" , Twist , queue_size=10)
        self.teleopcmd_subscriber = rospy.Subscriber("/cmd_teleop" , Twist , callback=self.teleop_callback)

        self.closest_distance = 999
        self.closest_obstacle = "nothing"
        self.closest_angle = 0
        self.stop_teleop = False
        self.angular_speed = 0.3


    def teleop_callback(self ,msg):

        if self.stop_teleop == False:
            self.cmd_publisher.publish(msg)


    def obstacle_callback(self ,msg):

        self.closest_distance = msg.distance
        self.closest_obstacle = msg.obstacle_name
        

    def laser_callback(self ,msg):

        self.closest_angle = self.find_min_angle(msg)

    def find_min_angle(self , laser_msg):

        min_distance = 999
        myangle = 0

        for i in range(360):

            if laser_msg.ranges[i] < min_distance:
                myangle = i
                min_distance = laser_msg.ranges[i]
        
        return myangle
            
    def turn(self):

        first_angle= self.closest_angle
        goal_angle = (150 + first_angle) % 360
        # print(first_angle)
        # print(goal_angle)

        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)

        while True :

            if (goal_angle - 5) <= self.closest_angle <= (goal_angle + 5):
                break



    def run(self):

        while not rospy.is_shutdown():

            if self.closest_distance < 1.5:

                # print(f"obstacle:{self.closest_obstacle} and distance:{self.closest_distance}")

                self.stop_teleop = True
                self.cmd_publisher.publish(Twist())

                self.turn()
                self.cmd_publisher.publish(Twist())
                self.stop_teleop = False
                sleep(2)
                # print(myangle)
            






if __name__ == "__main__":

    sensor = Sensor()

    sensor.run()