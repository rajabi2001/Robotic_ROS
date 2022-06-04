#!/usr/bin/python3

from time import sleep
from turtle import distance
import tf
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class Follower:
    
    def __init__(self) -> None:

        rospy.init_node("goal_follower" , anonymous=False)

        self.cmd_publisher = rospy.Publisher("/cmd_vel" , Twist , queue_size=10)

        self.default_distance = 0.3
        self.linear_speed = 0.12
        self.angular_speed = 0.2
        self.dt = 0.005
        self.goal_x = 3
        self.goal_y = -1

        # gains for angular PID Goal controller
        self.kp_g = 2.6
        self.ki_g = 0.001
        self.kd_g = 0.9

        # gains for PID Wall controller
        self.kp_w = 2
        self.ki_w = 0
        self.kd_w = 22
        # self.kp_l = 

        


    # def update_odom_object(self):
    #     self.odom_object = rospy.wait_for_message("/odom" , Odometry)


    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom#
        msg = rospy.wait_for_message("/odom" , Odometry)
        # msg = self.odom_object

        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw

    # position of the robot 
    def get_position(self):

        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        # msg = self.odom_object
        
        position = msg.pose.pose.position

        x, y = float("{:.1f}".format(position.x)), float("{:.1f}".format(position.y))
        return x ,y

    def Euclidean_distance(self, x1, x2, y1, y2):

        point1 = np.array((x1,y1))
        point2 = np.array((x2,y2))

        return np.linalg.norm(point1 - point2)

    def find_angle(self):

        robot_x ,robot_y = self.get_position()

        angle = float("{:.2f}".format(np.arctan2((self.goal_y - robot_y), (self.goal_x - robot_x))))

        return angle

    def get_angular_error(self):

        goal_orientaion = self.find_angle()
        robot_orientation = self.get_heading()
        if robot_orientation > 0:
            sign = -1 if (robot_orientation - math.pi < goal_orientaion < robot_orientation) else +1
        else:
            sign = +1 if (robot_orientation + math.pi > goal_orientaion > robot_orientation) else -1

        return sign * (math.pi - abs(abs(robot_orientation - goal_orientaion) - math.pi))


    def find_wall(self):

        min_distance = 999
        myangle = 0

        laser_msg = rospy.wait_for_message("/scan" , LaserScan)

        iterate_list = list(range(0,6)) + list(range(180,360))
        # iterate_list = list(range(0,360))
        for i in iterate_list:

            if laser_msg.ranges[i] < min_distance:
                myangle = i
                min_distance = laser_msg.ranges[i]
        
        return myangle , min_distance


    def turn_left(self):

        goal_angle = 270
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)

        while True :

            wall_angle ,wall_distance = self.find_wall()

            if (goal_angle - 3) <= wall_angle <= (goal_angle + 3):

                self.cmd_publisher.publish(Twist())
                sleep(1)
                break

    def turn_right(self):

        goal_angle = 270
        twist = Twist()
        twist.angular.z = -1 * self.angular_speed
        self.cmd_publisher.publish(twist)

        while True :

            wall_angle ,wall_distance = self.find_wall()

            if (goal_angle - 3) <= wall_angle <= (goal_angle + 3):

                self.cmd_publisher.publish(Twist())
                sleep(1)
                break   
         
    def wall_following(self):

        print("wall_following has started")
        twist = Twist()
        sum_angular_error = 0
        prev_angular_error = 0

        while True:

            robot_orientation = self.get_heading()
            print(robot_orientation)
            if -1.65 < robot_orientation < -1.48:
                print("fuuuuuuuuuuuck")
                twist = Twist()
                twist.linear.x = self.linear_speed 
                self.cmd_publisher.publish(twist)
                sleep(2)
                break

            wall_angle ,wall_distance = self.find_wall()
            distance_error = self.default_distance - wall_distance

            distance_error = self.default_distance - wall_distance
            sum_angular_error += (distance_error * self.dt)

            # P = self.kp_a * distance_error + abs(self.kp_l * angle_error)
            P = self.kp_w * distance_error
            I = self.ki_w * sum_angular_error
            D = self.kd_w * (distance_error - prev_angular_error) 

            twist.angular.z =  P + I + D
            # twist.linear.x = self.linear_speed - abs(self.kp_l * distance_error)
            twist.linear.x = self.linear_speed

            prev_angular_error = distance_error
            self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)


    def goal_following(self):


        twist = Twist()
        sum_angular_error = 0
        prev_angular_error = 0

        while True:

            
            wall_angle ,wall_distance = self.find_wall()
            if wall_distance <= 0.4:
                self.cmd_publisher.publish(Twist())
                sleep(1)
                break


            # self.update_odom_object()
            
            angular_error = self.get_angular_error()

            sum_angular_error += (angular_error * self.dt)


            # calculate PID for angular speed
            P = self.kp_g * angular_error
            I = self.ki_g * sum_angular_error
            D = self.kd_g * (angular_error - prev_angular_error) 
            twist.angular.z =  P + I + D

            prev_angular_error = angular_error

            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)

    def run(self):

        while not rospy.is_shutdown():

            self.goal_following()
            self.turn_left()
            self.wall_following()



if __name__ == "__main__":
    
    follower = Follower()


    follower.run()
    
    
    
    