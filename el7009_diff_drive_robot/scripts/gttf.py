#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import tf2_ros

class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_pub')
        self.sub = self.create_subscription(
            PoseStamped, '/ground_truth_pose', self.callback, 10)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        # Correcto: asignar campos individuales
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation  # Esto está bien, orientation ya es Quaternion
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GroundTruthTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
