#!/usr/bin/env python3

from turtle import distance
import numpy as np
from math import radians
from time import sleep
import matplotlib.pyplot as plt

import rospy
import tf


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.far_distance  = 1.2
        self.corner = 0
        self.errors = []

        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO


        # defining the rectangle
        X1 = np.linspace(-3, 3 , 100)
        Y1 = np.array([2]*100)

        Y2 = np.linspace(2, -2 , 100)
        X2 = np.array([3]*100)

        X3 = np.linspace(3, -3 , 100)
        Y3 = np.array([-2]*100)

        Y4 = np.linspace(-2, 2 , 100)
        X4 = np.array([-3]*100)

        X = np.concatenate([X1,X2,X3,X4]).tolist()
        Y = np.concatenate([Y1,Y2,Y3,Y4]).tolist()
        self.border = (X, Y)

        self.border_points = [(-3,2), (-3,-2), (3,-2), (3,2)]
        

    def next_corner(self):
        if self.corner == 3 :
            self.corner = 0
        else:
            self.corner += 1
        
        print(self.corner)

    def Euclidean_distance(self ,point1 ,point2):
        point1 = np.array(point1)
        point2 = np.array(point2)

        return np.linalg.norm(point1 - point2)

    def odom_callback(self, msg):

        pose = msg.pose.pose.position
        myposition = (pose.x, pose.y)
        border_distance = []

        for i in range(len(self.border_points)):
            border_distance.append(self.Euclidean_distance(myposition, self.border_points[i]))
        
        # print(border_distance[self.corner])
        self.errors.append(border_distance[self.corner])

        if border_distance[self.corner] >= self.far_distance:
            return
        
        
        if border_distance[self.corner] <= self.stop_distance: 
            self.state = self.ROTATE
            self.next_corner()
            print("small")
            return
        
        if (self.corner == 0 and (pose.x <= -3 or pose.y > 2.3)) or (self.corner == 1 and (pose.y <= -2 or pose.x < -3.3)) or (self.corner == 2 and (pose.x >= 3 or pose.y < -2.3) ) or (self.corner == 3 and (pose.y >= 2 or pose.x > 3.3)): 
            self.state = self.ROTATE
            self.next_corner()



 
    


    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw

    def get_position(self):

        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        position = msg.pose.pose.position

        return position

    def turn_90_left(self):

        remaining = self.goal_angle
        prev_angle = self.get_heading()
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
    
        # rotation loop
        while remaining > self.epsilon:
            
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
            
            
        

    def go_straight(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_publisher.publish(twist)

    def stop(self):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

    def start(self):
        
        rospy.sleep(1)
        self.turn_90_left()
        self.stop()
        self.go_straight()        

        while True:
            robot_pose = self.get_position()
            tmp_distance = self.Euclidean_distance((robot_pose.x,robot_pose.y),(0,2))

            if tmp_distance < 0.075 :
                self.stop()
                break
        
        self.turn_90_left()
        self.stop()

        self.state = self.GO

        
    def run(self):
        
        while not rospy.is_shutdown():

            if self.state == self.GO:
                self.go_straight()
                continue
            print("here")
            self.stop()

            self.turn_90_left()
            self.stop()

            self.state = self.GO
    
    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_publisher.publish(Twist())
        
        plt.plot(list(range(len(self.errors))),
                    self.errors, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.show()

        rospy.sleep(20)






if __name__ == "__main__":

    controller = Controller()
    
    sleep(5)
    print("hi everybody")
    controller.start()
    controller.run()

    