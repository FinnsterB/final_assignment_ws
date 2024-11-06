#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import time
# Import your custom message type
from smartcar_msgs.msg import Status
from nav_msgs.msg import Odometry

class VehicleStatusToTransform(Node):
    def __init__(self):
        super().__init__('wheel_odom_publisher')
        
        # Publisher for the JointState messages
        self.odom_pub = self.create_publisher(Odometry, '/smart_car/wheel/odom', 10)
        
        # Subscribe to the /smart_car/vehicle_status topic with the custom message type
        self.subscription = self.create_subscription(
            Status,                    # Custom message type
            '/smart_robot_car/vehicle_status',       # Topic name
            self.vehicle_status_callback,      # Callback function
            10                                 # QoS History depth
        )
        self.subscription  # prevent unused variable warning
        self.previousTime = None
        self.previousAngle = 0.0
        self.previousXPos = 0.0
        self.previousYPos = 0.0

    def vehicle_status_callback(self, msg):
        # Calculate deltaTime
        currentTime = self.get_clock().now()
        deltaTime = 0.0
        if self.previousTime is not None:
            deltaTime = (currentTime - self.previousTime).nanoseconds / 1e9  # Convert nanoseconds to mseconds
        self.previousTime = currentTime

        #TODO: Add uncertainty

        wheelDiameter = 0.064
        wheelBase = 0.257
        # Calculate linear velocity
        linearVelocity = (msg.engine_speed_rpm * math.pi * wheelDiameter)/ 60

        # Calculate angular velocity
        angularVelocity = (linearVelocity/wheelBase)* math.tan(msg.steering_angle_rad)

        angle = self.previousAngle + angularVelocity*deltaTime

        xPos = self.previousXPos + linearVelocity*math.cos(angle)*deltaTime

        yPos = self.previousYPos + linearVelocity*math.sin(angle)*deltaTime

        odomMsg = Odometry()

        odomMsg.pose.pose.position.x = xPos

        odomMsg.pose.pose.position.y = yPos

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, angle)



        odomMsg.pose.pose.orientation.x = quat[0]
        odomMsg.pose.pose.orientation.y = quat[1]
        odomMsg.pose.pose.orientation.z = quat[2]
        odomMsg.pose.pose.orientation.w = quat[3]

        odomMsg.twist.twist.linear.x = linearVelocity

        odomMsg.twist.twist.angular.z = angularVelocity

        # Publish the transform
        self.odom_pub.publish(odomMsg)

        print(angle)

        self.previousXPos = xPos
        self.previousYPos = yPos
        self.previousAngle = angle


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusToTransform()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
