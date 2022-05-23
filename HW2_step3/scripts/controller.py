#!/usr/bin/python3

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from time import sleep
import numpy as np
import math
from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
                

        # gains for angular PID controller
        self.kp_a = 1.75
        self.ki_a = 0.001
        self.kd_a = 0.3


        # gains for linear PID controller
        self.kp_l = 0.5
        self.ki_l = 0.001
        self.kd_l = 1.5


        # intial config
        self.dt = 0.05
        self.epsilon = 0.2
        self.errors = []

        # position and orientation of the robot
        self.odom_object = rospy.wait_for_message("/odom" , Odometry)
        self.pose_x = 1
        self.pose_y = 1
        self.orientaion = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_orientaion = 0

        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        # type of shape
        self.shape = 4
        self.X = []
        self.Y = []



    def fill_shape(self):

        if self.shape == 1:

            a = 0.17
            k = math.tan(a)
            self.X , self.Y = [] , []

            for i in range(150):
                t = i / 20 * math.pi
                dx = a * math.exp(k * t) * math.cos(t)
                dy = a * math.exp(k * t) * math.sin(t)
                self.X.append(dx)
                self.Y.append(dy)

        elif  self.shape == 2:

            X1 = np.linspace(-6., -2 , 50)
            Y1 = np.zeros((50,))

            x_dim, y_dim = 2,2
            t = np.linspace(np.pi, 0, 100)
            X2 = x_dim * np.cos(t) 
            Y2 = y_dim * np.sin(t)

            X3 = np.linspace(2, 6 , 50)
            Y3 = np.zeros((50,))

            x_dim, y_dim = 6,6
            t = np.linspace(np.pi*2, np.pi, 200)
            X4 = x_dim * np.cos(t) 
            Y4 = y_dim * np.sin(t)


            self.X = np.concatenate([X1,X2,X3,X4])
            self.Y = np.concatenate([Y1,Y2,Y3,Y4])

        elif self.shape == 3:

            X1 = np.linspace(-3, 3 , 100)
            Y1 = np.array([2]*100)

            Y2 = np.linspace(2, -2 , 100)
            X2 = np.array([3]*100)

            X3 = np.linspace(3, -3 , 100)
            Y3 = np.array([-2]*100)

            Y4 = np.linspace(-2, 2 , 100)
            X4 = np.array([-3]*100)

            self.X = np.concatenate([X1,X2,X3,X4])
            self.Y = np.concatenate([Y1,Y2,Y3,Y4])
        
        elif self.shape == 4:

            X1 = np.linspace(-1, 1 , 100)
            Y1 = np.array([3]*100)

            X2 = np.linspace(1, 1 + 2**(1/2) , 100)
            Y2 = - (2**(1/2)) * (X2 - 1) + 3

            Y3 = np.linspace(1, -1 , 100)
            X3 = np.array([1 + 2**(1/2)]*100)

            X4 = np.linspace(1 + 2**(1/2), 1, 100)
            Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 

            X5 = np.linspace(1, -1 , 100)
            Y5 = np.array([-3]*100)

            X6 = np.linspace(-1, -1 - 2**(1/2) , 100)
            Y6 = - (2**(1/2)) * (X6 + 1) - 3 


            Y7 = np.linspace(-1, 1 , 100)
            X7 = np.array([- 1 - 2**(1/2)]*100)


            X8 = np.linspace(-1 - 2**(1/2), -1, 100)
            Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1


            self.X = np.concatenate([X1,X2,X3,X4,X5,X6,X7,X8])
            self.Y = np.concatenate([Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8])





    def update_odom_object(self):
        self.odom_object = rospy.wait_for_message("/odom" , Odometry)


    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom#
        # msg = rospy.wait_for_message("/odom" , Odometry)
        msg = self.odom_object

        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw

    # position of the robot 
    def get_position(self):

        # waiting for the most recent message from topic /odom
        # msg = rospy.wait_for_message("/odom" , Odometry)
        msg = self.odom_object
        
        position = msg.pose.pose.position

        x, y = float("{:.1f}".format(position.x)), float("{:.1f}".format(position.y))
        return x ,y

    def Euclidean_distance(self, x1, x2, y1, y2):

        point1 = np.array((x1,y1))
        point2 = np.array((x2,y2))

        return np.linalg.norm(point1 - point2)

    def find_goal(self ,mode=0):

        my_min = 99999
        my_x = 0
        my_y = 0

        if mode == 0:
            for i in range(len(self.X)):

                my_distance = self.Euclidean_distance(self.pose_x, self.X[i], self.pose_y, self.Y[i])
                if my_distance < my_min:
                    my_min = my_distance
                    my_x = self.X[i]
                    my_y = self.Y[i]

        elif mode == 1:
            for i in range(len(self.X)):

                diff_ang = float("{:.2f}".format(np.arctan2((self.Y[i] - self.pose_y), (self.X[i] - self.pose_x))))

                if self.orientaion > 0:
                    sign = -1 if (self.orientaion - math.pi < diff_ang < self.orientaion) else +1
                else:
                    sign = +1 if (self.orientaion + math.pi > diff_ang > self.orientaion) else -1

                diff_ang =  sign * (math.pi - abs(abs(self.orientaion - diff_ang) - math.pi))

                if -math.pi/2 < diff_ang < math.pi/2:

                    my_distance = self.Euclidean_distance(self.pose_x, self.X[i], self.pose_y, self.Y[i])
                    if my_min > my_distance and my_distance > abs(self.epsilon):
                        my_min = my_distance
                        my_x = self.X[i]
                        my_y = self.Y[i]


        self.goal_x = my_x
        self.goal_y = my_y
        
        return my_min



    def get_angular_error(self):

        if self.orientaion > 0:
            sign = -1 if (self.orientaion - math.pi < self.goal_orientaion < self.orientaion) else +1
        else:
            sign = +1 if (self.orientaion + math.pi > self.goal_orientaion > self.orientaion) else -1

        return sign * (math.pi - abs(abs(self.orientaion - self.goal_orientaion) - math.pi))

    def get_linear_error(self):
        
        mode = 0
        if self.find_goal() < self.epsilon:
            mode = 1

        self.find_goal(mode=mode)
        self.goal_orientaion =  float("{:.2f}".format(np.arctan2((self.goal_y - self.pose_y), (self.goal_x - self.pose_x))))
        linear_error = self.find_goal(mode=mode)

        return linear_error

        


    def PID_controller(self):

        twist = Twist()
        sum_angular_error = 0
        sum_linear_error = 0
        prev_angular_error = 0
        prev_linear_error = 0

        while not rospy.is_shutdown():



            self.update_odom_object()
            self.orientaion = self.get_heading()
            self.pose_x, self.pose_y = self.get_position()
            

            linear_error = self.get_linear_error()
            angular_error = self.get_angular_error()
            
            
            self.errors.append(linear_error)
            sum_angular_error += (angular_error * self.dt)
            sum_linear_error += (linear_error * self.dt)

            # calculate PID for linear speed
            P = self.kp_l * linear_error
            I = self.ki_l * sum_linear_error
            D = self.kd_l * (linear_error - prev_linear_error)
            twist.linear.x =  P + I + D

            # calculate PID for angular speed
            P = self.kp_a * angular_error
            I = self.ki_a * sum_angular_error
            D = self.kd_a * (angular_error - prev_angular_error) 
            twist.angular.z =  P + I + D

            prev_angular_error = angular_error
            prev_linear_error = linear_error

            self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)

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
    controller.fill_shape()
    controller.PID_controller()
