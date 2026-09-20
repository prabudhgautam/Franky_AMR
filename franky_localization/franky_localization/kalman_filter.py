#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class KalmanFilter(Node):

    def __init__(self):
        super().__init__("kalman_filter")
        self.odom_sub_ = self.create_subscription(Odometry, "franky_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "franky_controller/odom_kalman", 10)
        
        # Initially the robot has no idea about how fast is going
        self.mean_ = 0.0
        self.variance_ = 1000.0

        # Modeling the uncertainty of the sensor and the motion
        self.motion_variance_ = 4.0  #variance of robot motion
        self.measurement_variance_ = 0.5 #variance of imu sensor

        # Store the messages - only for the orientation
        self.imu_angular_z_ = 0.0

        self.is_first_odom_ = True

#angular vel about z axis, using encodr meawsurment as prior belief
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0

        # Publish the filtered odometry message
        self.kalman_odom_ = Odometry()

    def imuCallback(self, imu):
            # Store the measurement update
            self.imu_angular_z_ = imu.angular_velocity.z

#gives Posterior = updated state after measurement
#This mean will contgain the new estimate of the angular vel. of the robot,acccprding to the 1D KF Algo.
    def measurementUpdate(self):
            self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) \
                       / (self.variance_ + self.measurement_variance_)
            #New variance related to just calculated mean , both of them together will represnt the new Gaussian Distribution of estimate of angular vel. pof robot absed on new measurs(new evidance coming from imu)             
            self.variance_ = (self.variance_ * self.measurement_variance_) \
                           / (self.variance_ + self.measurement_variance_)

#This is not a Gaussian fusion formula. It is a dynamics update.
#gives Prior = predicted state before measurement
    def statePrediction(self):
            self.mean_ = self.mean_ + self.motion_
            self.variance_ = self.variance_ + self.motion_variance_
    
    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.is_first_odom_ = False
            self.mean_ = odom.twist.twist.angular.z
            return
        
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        self.statePrediction()
        self.measurementUpdate()

        # Update for the next iteration
        self.last_angular_z_ = odom.twist.twist.angular.z

        # Update and publish the filtered odom message
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)


def main():
    rclpy.init()

    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    
    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    