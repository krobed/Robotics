#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        with open('el7009_diff_drive_robot/worlds/box_obstacle.sdf', 'r') as f:
            sdf_xml = f.read()

        request = SpawnEntity.Request()
        request.name = 'box_obstacle'
        request.xml = sdf_xml
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = -2.0
        request.initial_pose.position.z = 0.0

        self.future = self.cli.call_async(request)

def main():
    rclpy.init()
    node = ObstacleSpawner()
    rclpy.spin_until_future_complete(node, node.future)
    if node.future.result() is not None:
        node.get_logger().info('Spawned successfully!')
    else:
        node.get_logger().error('Failed to spawn.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
