#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
import math

class OdomPublisher4Wheel(Node):
    def __init__(self):
        super().__init__('odom')
        
        # --- CONFIGURATION ---
        self.wheel_radius = 0.05       
        self.wheel_separation = 0.26    
        
        self.left_front_name = 'left_wheel_front_joint'
        self.left_back_name = 'left_wheel_back_joint'
        self.right_front_name = 'right_wheel_front_joint'
        self.right_back_name = 'right_wheel_back_joint'
        # ---------------------

        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.prev_pos = {} 
        self.last_time = self.get_clock().now()
        self.first_run = True
        
        print("Odom node UP and Running!!")

    def joint_callback(self, msg):
        try:
            lf_idx = msg.name.index(self.left_front_name)
            lb_idx = msg.name.index(self.left_back_name)
            rf_idx = msg.name.index(self.right_front_name)
            rb_idx = msg.name.index(self.right_back_name)
        except ValueError:
            return

        current_pos = {
            'lf': msg.position[lf_idx],
            'lb': msg.position[lb_idx],
            'rf': msg.position[rf_idx],
            'rb': msg.position[rb_idx]
        }
        
        current_time = self.get_clock().now()

        # Handle First Run
        if self.first_run:
            self.prev_pos = current_pos
            self.last_time = current_time
            self.first_run = False
            return

        # --- FIX IS HERE: Calculate dt and Check for Zero ---
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt < 0.0001: 
            # Time hasn't changed enough (or is zero). 
            # Skip this loop to prevent DivisionByZero and Infinite Velocity.
            return
            
        self.last_time = current_time
        # ----------------------------------------------------

        # 1. Calculate rotation delta for each wheel
        d_lf = (current_pos['lf'] - self.prev_pos['lf']) * self.wheel_radius
        d_lb = (current_pos['lb'] - self.prev_pos['lb']) * self.wheel_radius
        d_rf = (current_pos['rf'] - self.prev_pos['rf']) * self.wheel_radius
        d_rb = (current_pos['rb'] - self.prev_pos['rb']) * self.wheel_radius
        
        self.prev_pos = current_pos

        # 2. Average Left and Right
        d_left = (d_lf + d_lb) / 2.0
        d_right = (d_rf + d_rb) / 2.0

        # 3. Calculate Linear and Angular Displacement
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # 4. Integrate Position
        if abs(d_theta) < 0.000001:
            self.x += d_center * math.cos(self.th + (d_theta / 2.0))
            self.y += d_center * math.sin(self.th + (d_theta / 2.0))
        else:
            radius = d_center / d_theta
            self.x += radius * (math.sin(self.th + d_theta) - math.sin(self.th))
            self.y -= radius * (math.cos(self.th + d_theta) - math.cos(self.th))

        self.th += d_theta
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # 5. Calculate Velocities (This is where it crashed before)
        vx = d_center / dt
        vth = d_theta / dt

        # --- PUBLISH ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.th)
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        odom.pose.covariance[0] = 0.00001
        odom.pose.covariance[7] = 0.00001
        odom.pose.covariance[35] = 0.001

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher4Wheel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()