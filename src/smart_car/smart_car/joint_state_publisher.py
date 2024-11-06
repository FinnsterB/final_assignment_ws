#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
from smartcar_msgs.msg import Status

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Publisher for the JointState messages
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribe to the /smart_car/vehicle_status topic with the custom message type
        self.subscription = self.create_subscription(
            Status,                    # Custom message type
            '/smart_robot_car/vehicle_status',       # Topic name
            self.vehicle_status_callback,      # Callback function
            10                                 # QoS History depth
        )
        self.subscription  # prevent unused variable warning
        self.previousTime = None

        # Define joint names
        self.joint_names = ['front_left_wheel_steer_joint', 
                            'front_right_wheel_steer_joint', 
                            'front_left_wheel_joint', 
                            'front_right_wheel_joint',
                            'back_left_wheel_joint', 
                            'back_right_wheel_joint']
        
        # Define initial positions, velocities, and efforts for each joint
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_efforts = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Start time to simulate joint movement over time
        self.start_time = time.time()

    def vehicle_status_callback(self, msg):
        # Calculate deltaTime
        currentTime = self.get_clock().now()
        deltaTime = 0.0
        if self.previousTime is not None:
            deltaTime = (currentTime - self.previousTime).nanoseconds / 1e9  # Convert nanoseconds to mseconds
        self.previousTime = currentTime

        # Create a new JointState message
        joint_state_msg = JointState()

        # Populate the JointState message with names, positions, velocities, and efforts
        joint_state_msg.name = self.joint_names
        
        # Simulate joint positions in a cyclic pattern
        current_time = time.time() - self.start_time
        self.joint_positions[0] = msg.steering_angle_rad
        self.joint_positions[1] = msg.steering_angle_rad


        self.joint_positions[4] = self.joint_positions[4] + (msg.engine_speed_rpm*60)*deltaTime
        self.joint_positions[5] = self.joint_positions[5] + (msg.engine_speed_rpm*60)*deltaTime
        joint_state_msg.position = self.joint_positions
        
        # For this example, set velocities and efforts to zero
        joint_state_msg.velocity = self.joint_velocities
        joint_state_msg.effort = self.joint_efforts

        # Add the current time as the message timestamp
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the JointState message
        self.joint_pub.publish(joint_state_msg)
        #self.get_logger().info('Publishing joint states: %s' % joint_state_msg.position)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass

    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
