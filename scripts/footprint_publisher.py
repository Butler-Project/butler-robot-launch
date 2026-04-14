#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

import math

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.
    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('footprint_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 30.0)
        
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
        self.footprint_frame = self.get_parameter('footprint_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        self.get_logger().info(f"Footprint Publisher started: {self.odom_frame} -> {self.footprint_frame} (sibing of {self.base_link_frame})")

    def timer_callback(self):
        try:
            # Look up transform from odom to base_link
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_link_frame,
                rclpy.time.Time())
                
            # Extract position
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Extract yaw from quaternion
            q = trans.transform.rotation
            (roll, pitch, yaw) = euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # Create new transform for footprint (x, y, 0) and (0, 0, yaw)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.footprint_frame
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0 # On ground
            
            q_new = quaternion_from_euler(0, 0, yaw)
            t.transform.rotation.x = q_new[0]
            t.transform.rotation.y = q_new[1]
            t.transform.rotation.z = q_new[2]
            t.transform.rotation.w = q_new[3]
            
            # Send the transform
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            # It's normal to have exceptions at startup before TF is available
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FootprintPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
