#!/usr/bin/env python3
#2021 Hotae Lee <hotae.lee@berkeley.edu>

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node

from ros2_connect.msg import StateEst, NpcState, NpcStateArray


class ConnectNode(Node):
    def __init__(self):
        super().__init__('connector')
        self.pub = self.create_publisher(NpcStateArray, 'carla/npc_state_array2nuvo', qos_profile_sensor_data)
        timer_period = 0.5
        # self.i = 0
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.sub = self.create_subscription(NpcStateArray, 'carla/npc_state_array', self.sub_callback, qos_profile_sensor_data)
        self.npcs_carla = NpcStateArray()
    def sub_callback(self, msg):
        self.get_logger().info('i heard: "%s"' %msg.header)
        self.get_logger().info('i heard: "%f"' %msg.npc_states[0].loc.x)
        self.npcs_carla.npc_states = msg.npc_states
        print('check')
    def pub_callback(self):
        msg = NpcStateArray()
        msg = self.npcs_carla
        self.pub.publish(msg)
        # self.i += 1

def main(args=None):
    rclpy.init(args=args)
    connect_node = ConnectNode()
    rclpy.spin(connect_node)

    connect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()