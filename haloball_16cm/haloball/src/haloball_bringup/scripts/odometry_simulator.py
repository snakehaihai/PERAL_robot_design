#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Point, Vector3
import tf2_ros
import math
from tf_transformations import quaternion_from_euler

class Simulator(Node):
    def __init__(self):
        super().__init__('odometry_simulator')

        # Orbit parameters
        self.radius = 0.089        # radius of orbit (m)
        self.angular_speed = 0.5  # rad/s
        self.time = 0.0
        
        # X-axis motion parameters
        self.x_speed = 0.2        # linear speed along x-axis (m/s)
        self.x_direction = 1      # 1 for positive, -1 for negative
        self.x_limit = 3        # limit for x-axis motion (m)
        
        # State machine variables
        self.phase = 1  # 1: forward to right, 2: back to center, 3: forward to left, 4: back to center
        self.current_theta = 0.0
        self.target_theta = math.pi/2  # First target: 90 degrees (quarter circle)
        
        # Current x position
        self.current_x = 0.0
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.update_simulation)
        self.get_logger().info('Odometry simulator started')

    def update_simulation(self):
        dt = 0.1
        
        # Update X-axis position
        self.current_x += self.x_speed * self.x_direction * dt
        
        # Reverse X direction if limits are reached
        if self.current_x >= self.x_limit:
            self.current_x = self.x_limit
            self.x_direction = -1
        elif self.current_x <= -self.x_limit:
            self.current_x = -self.x_limit
            self.x_direction = 1

        # State machine for back-and-forth circular motion
        if self.phase == 1:  # Forward to right quarter circle
            self.current_theta += self.angular_speed * dt
            if self.current_theta >= self.target_theta:
                self.current_theta = self.target_theta
                self.phase = 2
                self.target_theta = 0.0
                
        elif self.phase == 2:  # Back to center from right
            self.current_theta -= self.angular_speed * dt
            if self.current_theta <= self.target_theta:
                self.current_theta = self.target_theta
                self.phase = 3
                self.target_theta = -math.pi/2  # -90 degrees
                
        elif self.phase == 3:  # Forward to left quarter circle
            self.current_theta -= self.angular_speed * dt
            if self.current_theta <= self.target_theta:
                self.current_theta = self.target_theta
                self.phase = 4
                self.target_theta = 0.0
                
        elif self.phase == 4:  # Back to center from left
            self.current_theta += self.angular_speed * dt
            if self.current_theta >= self.target_theta:
                self.current_theta = self.target_theta
                self.phase = 1  # Restart cycle
                self.target_theta = math.pi/2

        # Calculate circular position using circle equations
        x_circular = self.radius * math.sin(self.current_theta)
        y = 0.0
        z = -self.radius * (1 - math.cos(self.current_theta))

        # Combine circular motion with X-axis motion
        x = self.current_x + x_circular

        # Calculate velocities based on direction
        if self.phase == 1 or self.phase == 3:
            # Forward motion phases
            direction = 1 if self.phase == 1 else -1
            dx_dt_circular = self.radius * self.angular_speed * math.cos(self.current_theta) * direction
            dz_dt = self.radius * self.angular_speed * math.sin(self.current_theta) * direction
        else:
            # Reverse motion phases
            direction = -1 if self.phase == 2 else 1
            dx_dt_circular = self.radius * self.angular_speed * math.cos(self.current_theta) * direction
            dz_dt = self.radius * self.angular_speed * math.sin(self.current_theta) * direction

        # Add X-axis velocity to circular velocity
        dx_dt = self.x_speed * self.x_direction + dx_dt_circular

        # Calculate orientation
        roll = 0.0
        yaw = 0.0
        
        # Pitch based on movement direction (circular motion only for orientation)
        if self.phase == 1 or self.phase == 4:
            # Forward motion - face direction of travel
            pitch = math.atan2(dz_dt, dx_dt_circular)
        else:
            # Reverse motion - face opposite direction
            pitch = math.atan2(dz_dt, dx_dt_circular) + math.pi
        
        # Normalize pitch to [-π, π]
        if pitch > math.pi:
            pitch -= 2 * math.pi
        elif pitch < -math.pi:
            pitch += 2 * math.pi

        # Linear velocity (combined)
        linear_vel_x = dx_dt
        linear_vel_y = 0.0
        linear_vel_z = dz_dt

        # Publish odometry
        self.publish_odometry(x, y, z, roll, pitch, yaw,
                              linear_vel_x, linear_vel_y, linear_vel_z)
        # Publish TF
        self.publish_transform(x, y, z, roll, pitch, yaw)
        
        # Log current state for debugging
        # self.get_logger().info(f'Phase: {self.phase}, Theta: {self.current_theta:.2f}, X: {self.current_x:.2f}, Position: ({x:.2f}, {y:.2f}, {z:.2f})')

    def publish_odometry(self, x, y, z, roll, pitch, yaw,
                         vx, vy, vz):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'camera_init'
        odom_msg.child_frame_id = 'body'
        
        # Position
        odom_msg.pose.pose.position = Point(x=x, y=y, z=z)
        q = quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )
        
        # Velocity
        odom_msg.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.odom_pub.publish(odom_msg)

    def publish_transform(self, x, y, z, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'body'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()