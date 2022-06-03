#!/usr/bin/python3

from time import sleep
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Follower:
    
    def __init__(self) -> None:

        rospy.init_node("maze_follower" , anonymous=False)

        self.cmd_publisher = rospy.Publisher("/cmd_vel" , Twist , queue_size=10)

        self.default_distance = 0.5
        self.linear_speed = 0.2
        self.angular_speed = 0.2
        self.dt = 0.005
        self.epsilon = 2

        # gains for PID controller
        self.kp_a = 2
        self.ki_a = 0
        self.kd_a = 25
        self.kp_l = 0.0005

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

            wall_angle ,wall_distance = self.find_wall()
            angle_error = (270 - wall_angle) % 180
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






if __name__ == "__main__":
    
    follower = Follower()

    # follower.reach_wall_first_time()
    # follower.wall_following()
    
    
    
    