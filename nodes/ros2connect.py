#!/usr/bin/env python3
#2021 Hotae Lee <hotae.lee@berkeley.edu>

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node

from ros2_connect.msg import StateEst, NpcState, NpcStateArray, SPaT, SPaTArray
from arpae_common_msgs.msg import StateEst, NpcState, NpcStateArray, SPaT, SPaTArray
# from localization_msgs.msg import Obstacle

class ConnectNode(Node):
    def __init__(self):
        super().__init__('connector')
        """Publisher"""
        self.pub = self.create_publisher(NpcStateArray, 'carla/npc_state_array2nuvo', qos_profile_sensor_data)
        self.pub2 = self.create_publisher(SPaTArray, 'carla/spats2nuvo', qos_profile_sensor_data)
        self.pub_cohda = self.create_publisher(SPaT, 'cohda/spat2carla', 10)
        timer_period = 0.1
        # self.i = 0
        self.timer = self.create_timer(timer_period, self.pub_callback)
        
        """Subscriber"""
        # From ROS1 CARLA
        self.sub = self.create_subscription(NpcStateArray, 'carla/npc_state_array', self.sub_callback, qos_profile_sensor_data)
        self.sub2 = self.create_subscription(SPaTArray, 'carla/spats', self.sub2_callback, qos_profile_sensor_data)
        # From ROS2 Cohda
        self.sub_cohda = self.create_subscription(SPaT, '/localization/matched_obstacle_carla', self.sub_cohda_callback, qos_profile_sensor_data)
        self.npcs_carla = NpcStateArray()
        self.spats_carla = SPaTArray()
        self.spat_cohda = SPaT()
    def sub_callback(self, msg):
        self.get_logger().info('i heard: "%s"' %msg.header)
        # self.get_logger().info('i heard: "%f"' %msg.npc_states[0].loc.x)
        self.npcs_carla.npc_states = msg.npc_states
        print('check')
    def sub2_callback(self, msg):
        self.get_logger().info('i heard: "%s"' %msg.header)
        # self.get_logger().info('i heard: "%f"' %msg.npc_states[0].loc.x)
        self.spats_carla.spats = msg.spats
        print('check')
    def sub_cohda_callback(self, msg):
        self.spat_cohda.intersection_id = msg.intersection_id
        self.spat_cohda.obstacle_free = msg.obstacle_free
        self.spat_cohda.signal_phase = msg.signal_phase
        self.spat_cohda.signal_timing = msg.signal_timing
        self.spat_cohda.stop_bar_distance = msg.stop_bar_distance
    def pub_callback(self):
        # Publish CARLA NPCs to NUVO
        msg = NpcStateArray()
        msg = self.npcs_carla
        self.pub.publish(msg)
        # Publish CARLA SPaTs to NUVO
        spat_msg = SPaTArray()
        spat_msg = self.spats_carla
        self.pub2.publish(spat_msg)
        # Convert Cohda msg into SPaT msg and Publish it to CARLA(ROS1)
        cohda_msg = SPaT()
        print("what does empty SPaT have?:{}".format(cohda_msg))
        cohda_msg = self.spat_cohda
        self.pub_cohda.publish(cohda_msg)


def main(args=None):
    rclpy.init(args=args)
    connect_node = ConnectNode()
    rclpy.spin(connect_node)

    connect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()