#!/usr/bin/python3

import tf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class VFH_Controller():

    def __init__(self):

        rospy.init_node('vfh', anonymous=False)

        self.target_x = -8
        self.target_y = -8

        self.angular_speed = 0.1
        self.angular_epsilon = 0.1
        self.linear_spead = 0.1
        self.linear_epsilon = 0.05
        self.linear_lenght = 1

        self.sector_size = 5
        self.a = 1
        self.b = 0.25
        self.threshold = 2.5
        self.s_max = 9


        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


    def get_laser_scan(self):
        self.laser_scan = rospy.wait_for_message("/scan", LaserScan)

    def get_pose(self):
        odom = rospy.wait_for_message("/odom", Odometry)
        orientation = odom.pose.pose.orientation
        position = odom.pose.pose.position
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        ))

        return position, yaw

    
    def smoothing_histogram(self,sectors):

        smoothed = []
        for i in range(self.number_of_sector):
            sum_h = 0
            for j in range(-2,3):
                
                if j == 2 or j == -2:
                    jj = 1
                else:
                    jj = 2
                
                if i+j >= self.number_of_sector:
                    j = j * -1
                    i  = self.number_of_sector - i - 1

                sum_h += sectors[i+j] * jj

            sum_h = sum_h/5
            smoothed.append(sum_h)
        
        return smoothed

    def thresholding(self ,sectors):

        thresholded = []
        for i in range(self.number_of_sector):
            
            if sectors[i] < self.threshold:
                thresholded.append(i)
        
        return thresholded



    def find_target_sector(self):

        position, yaw = self.get_pose()
        angle = math.atan2(self.target_x - position.y, self.target_y - position.x)

        if angle < 0:
            angle += 2 * math.pi

        dif = angle - yaw

        if dif < 0:
            dif += 2 * math.pi
        
        target_index = int(math.degrees(dif) / self.sector_size) 

        return target_index % self.number_of_sector
        

    def calculate_Histogram(self):
       histogram = []
       self.get_laser_scan()

       
       self.number_of_sector = int(len(self.laser_scan.ranges)/self.sector_size)
       
       for i in range(self.number_of_sector):
           
           tmp_histogram = 0
           for j in range(i*self.sector_size,(i+1)*self.sector_size):
               magnitude = self.a - self.b * min(6,self.laser_scan.ranges[j])
               tmp_histogram += magnitude
           
           histogram.append(tmp_histogram)
    
       
       return self.smoothing_histogram(histogram)
    #    return histogram


    def vallye_clstering(self ,selected_sectors):

        valleys = []
        tmp_valley=[]
        # print(selected_sectors)
        for i in range(len(selected_sectors)):

            j = i - 1

            if i == 0 :
                tmp_valley.append(selected_sectors[i])
                continue
            
            if selected_sectors[i] - selected_sectors[j] > 1:
                valleys.append(tmp_valley)
                tmp_valley = []

            tmp_valley.append(selected_sectors[i])

        valleys.append(tmp_valley)
        tmp_valley = []

        if valleys[0][0] == 0 and  valleys[-1][-1] == (self.number_of_sector -1):
            tmp_valley = valleys.pop(0)
            for i in tmp_valley:
                valleys[-1].append(i)
            

        print(valleys)
        return valleys

    def select_valley(self,selected_sectors,target_sector):

        my_min= 999
        my_index = 0
        # my_index2 = 0

        
        valleys = self.vallye_clstering(selected_sectors)
        # print(valleys)
        for i in range(len(valleys)):

            for j in range(len(valleys[i])):
                
                distances = abs(valleys[i][j]  - target_sector)

                if distances > 36:
                    distances = 72 - distances

                # print(str(valleys[i][j]) +' '+str(distances))

                if distances < my_min:
                    my_min = distances
                    my_index = i
                    
                    # my_index2 = j

        
        closest_valley = valleys[my_index]
        # print(valleys[my_index][my_index2])
        
        if len(closest_valley) <= self.s_max:
            return closest_valley[int(len(closest_valley)/2)]
        else :
            return closest_valley[my_index+int(self.s_max/2)]
        

    def controller(self, angle):

        remaining = angle
        prev_angle = self.get_pose()[1]

        rospy.sleep(1)
        sign = 1
        if angle > math.pi:
            angle -= 2 * math.pi
        if angle < -math.pi:
            angle += 2 *math.pi
        if angle < 0:
            sign = -1

        twist = Twist()
        twist.angular.z = sign * self.angular_speed
        self.cmd_vel.publish(twist)

        while abs(remaining) >= self.angular_epsilon:

            current_angle =self.get_pose()[1]
            delta = current_angle - prev_angle

            if abs(delta) < 0.2:
                remaining -= delta
            prev_angle = current_angle


        twist.angular.z = 0
        self.cmd_vel.publish(twist)
        # rospy.sleep(1)

        remaining = self.linear_lenght
        prev_position = self.get_pose()[0]

        rospy.sleep(1)

        twist = Twist()
        twist.linear.x = self.linear_spead
        self.cmd_vel.publish(twist)

        while remaining >= self.linear_epsilon:
            current_position =self.get_pose()[0]
            delta = np.linalg.norm([current_position.x - prev_position.x, current_position.y - prev_position.y])
            remaining -= delta
            remaining = abs(remaining)
            prev_position = current_position
            
        twist.linear.x = 0
        self.cmd_vel.publish(twist)
        rospy.sleep(1)

        self.cmd_vel.publish(Twist())



    def global_path_planning(self):

        sectors = self.calculate_Histogram()
        
        print(sectors[-5:])
        print(sectors[:5])
        
        target_sector = self.find_target_sector()
        selected_sectors = self.thresholding(sectors)

        if sectors[target_sector] < self.threshold:
            best_sector = target_sector
        else:
            best_sector = self.select_valley(selected_sectors, target_sector)

        if best_sector > 36:
            best_sector -= 72

        print(best_sector)
        angle = math.radians(best_sector * 5)

        self.controller(angle)

        



if __name__ == '__main__':
       
    vfh = VFH_Controller()

    while not rospy.is_shutdown():

        vfh.global_path_planning()
        # rospy.sleep(10)


    
    