#!/usr/bin/python3

from time import sleep
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Follower:
    
    def __init__(self) -> None:

        rospy.init_node("wall_follower" , anonymous=False)

        self.cmd_publisher = rospy.Publisher("/cmd_vel" , Twist , queue_size=10)

        self.default_distance = 0.5
        self.linear_speed = 0.2
        self.angular_speed = 0.2

    def find_wall(self):

        min_distance = 999
        myangle = 0

        laser_msg = rospy.wait_for_message("/scan" , LaserScan)

        for i in range(360):

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

            if (goal_angle - 2) <= wall_angle <= (goal_angle + 2):

                self.cmd_publisher.publish(Twist())
                break

    def reach_wall_first_time(self):

        twist = Twist()
        twist.linear.x = self.linear_speed

        while True:

            self.cmd_publisher.publish(twist)

            wall_angle ,wall_distance = self.find_wall()

            if wall_distance <= self.default_distance:

                print(f"angle:{wall_angle} and distance:{wall_distance}")
                self.cmd_publisher.publish(Twist())
                break
        
        self.turn_left()
        

            



if __name__ == "__main__":
    
    follower = Follower()

    # in order to reach the wall
    follower.reach_wall_first_time()
    
    
    
    