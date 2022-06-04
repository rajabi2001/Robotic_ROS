#!/usr/bin/python3

from time import sleep
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class Follower:
    
    def __init__(self) -> None:

        rospy.init_node("goal_follower" , anonymous=False)

        self.cmd_publisher = rospy.Publisher("/cmd_vel" , Twist , queue_size=10)

        self.default_distance = 0.375
        self.linear_speed = 0.15
        self.angular_speed = 0.2
        self.dt = 0.005
        self.goal_x = 3
        self.goal_y = -1

        # gains for PID controller
        self.kp_a = 3
        self.ki_a = 0
        self.kd_a = 35


    def get_position(self):

        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        position = msg.pose.pose.position

        x, y = float("{:.1f}".format(position.x)), float("{:.1f}".format(position.y))
        return x ,y

    def find_wall(self):

        min_distance = 999
        myangle = 0

        laser_msg = rospy.wait_for_message("/scan" , LaserScan)

        iterate_list = list(range(0,6)) + list(range(180,360))
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
                break

    def reach_wall_first_time(self):

        twist = Twist()
        twist.linear.x = self.linear_speed

        while True:

            self.cmd_publisher.publish(twist)

            wall_angle ,wall_distance = self.find_wall()

            if wall_distance <= (self.default_distance + 0.05):

                print(f"angle:{wall_angle} and distance:{wall_distance}")
                self.cmd_publisher.publish(Twist())
                break
        
        self.turn_left()
        
        twist = Twist()
        twist.linear.x = 0.5
        self.cmd_publisher.publish(Twist())
        sleep(1)
        

            
    def wall_following(self):

        print("wall_following has started")
        twist = Twist()
        sum_angular_error = 0
        prev_angular_error = 0

        while not rospy.is_shutdown():

            checklist = list(range(0,5)) + list(range(355,360))

            wall_angle ,wall_distance = self.find_wall()
            # print(f"{wall_angle} , {wall_distance}")
            if wall_angle in checklist:
                
                self.turn_left()
                twist = Twist()
                twist.linear.x = 0.1
                self.cmd_publisher.publish(Twist())
                sleep(1)
                continue

            distance_error = self.default_distance - wall_distance

            distance_error = self.default_distance - wall_distance
            sum_angular_error += (distance_error * self.dt)

            # P = self.kp_a * distance_error + abs(self.kp_l * angle_error)
            P = self.kp_a * distance_error
            I = self.ki_a * sum_angular_error
            D = self.kd_a * (distance_error - prev_angular_error) 

            twist.angular.z =  P + I + D
            # twist.linear.x = self.linear_speed - abs(self.kp_l * angle_error)
            twist.linear.x = self.linear_speed

            prev_angular_error = distance_error
            self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)




    def reach_goal(self):


        pass

if __name__ == "__main__":
    
    follower = Follower()

    # follower.reach_wall_first_time()
    # follower.wall_following()
    
    
    
    