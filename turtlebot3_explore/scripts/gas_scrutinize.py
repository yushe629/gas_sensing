#!/usr/bin/env python
import imp
import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
from turtlebot3_explore.msg import Gas

# if scan_val is Inf, inf_distance is assigned.
inf_distance = 5.0
# the radius of the robot explore area
# territory_radius = 1.0  for aibot
half_of_scan_size = 20
# the velocity of exploring and the type of explore state
explore_vel = 0.3
explore_time = 0.3
explore_yaw_vel = 1.0
explore_yaw_time = 0.3
explore_state = ['front', 'back', 'turn', 'after_turn', 'explored']

class gas_scrutinize:
    def __init__(self):

        rospy.init_node("gas_scrutinize")

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.gas_value_sub = message_filters.Subscriber("/gas", Gas)
        self.robot_pose_sub = message_filters.Subscriber("/odom", Odometry)
        self.delay = 1/100. * 0.5
        self.mf = message_filters.ApproximateTimeSynchronizer([self.gas_value_sub, self.robot_pose_sub], 10, self.delay)
        self.mf.registerCallback(self.gas_odom_callback)

        self.territory_radius = rospy.get_param("~territory_radius", 0.6)
        
        self.is_finish_search_sub = rospy.Subscriber("/is_finish_search", Bool, self.search_callback)

        self.execute = False

        # self.estimated_gas_map_sub = rospy.Subscriber("estimated_gas_map", self.map_callback)

        self.timeout_sec = rospy.get_param("~timeout_sec", 15.0)

        self.start_time = 0
        self.max_gas_value = 0.0
        self.final_robot_pose = Pose()
        self.robot_pose = Pose()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.cmd_x = 0.0
        self.cmd_yaw = 0.0
        self.explore_state = 'front'
        self.iscorrect_path = True
        self.max_gas_value_time = rospy.get_time()
        self.before_gas_value = 0
        self.gas_value = 0

        rospy.spin()

    def search_callback(self, msg):
        self.execute = msg.data
        self.start_time = rospy.get_time()

    def gas_odom_callback(self, gas_msg, odom_msg):
        if not self.execute:
            return
        self.gas_value = gas_msg.gas.data
        self.before_gas_value = self.gas_value
        self.gas_value = gas_msg.gas.data
        if gas_msg.gas.data > self.max_gas_value:
            self.max_gas_value = gas_msg.gas.data
            self.final_robot_pose = odom_msg.pose.pose
            self.max_gas_value_time = rospy.get_time()

    # def odom_callback(self, msg):
    #     if not self.execute:
    #         return
    #     self.robot_pose = msg.pose.pose

    # def gas_callback(self,msg):
    #     if not self.execute:
    #         return
    #     self.before_gas_value = self.gas_value
    #     self.gas_value = msg.data
    #     if msg.data > self.max_gas_value:
    #         self.max_gas_value = msg.data
    #         self.final_robot_pose = self.robot_pose
    #         self.max_gas_value_time = rospy.get_time()


    def explore(self):
        if self.explore_state == 'front':
            self.cmd_x = explore_vel
            self.cmd_yaw = 0.0
            self.explore_state = 'back'
        elif self.explore_state == 'back':
            rospy.sleep(explore_time)
            self.cmd_x = -1.0*explore_vel
            self.explore_state = 'turn'
        elif self.explore_state == 'turn':
            rospy.sleep(explore_time)
            self.cmd_x = 0.0
            self.cmd_yaw = explore_yaw_vel
            self.explore_state = 'after_turn'
        elif self.explore_state == 'after_turn':
            rospy.sleep(explore_yaw_time)
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            self.explore_state = 'front'
            
    def callback(self, msg):
        if not self.execute:
            return

        if self.start_time == 0:
            self.start_time = rospy.get_time()
            self.max_gas_value_time = rospy.time()
            return

        if self.explore_state == "explored":
            return

        last_sec = (rospy.get_time() - self.max_gas_value_time)
        # rospy.loginfo_throttle(1.0, "last_sec: %f", last_sec)
        if last_sec  > self.timeout_sec:
            rospy.logwarn_once("Robot discovered goal!")
            self.explore_state = "explored"
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            self.goal_pose.header.seq = self.goal_pose.header.seq + 1
            self.goal_pose.header.stamp = rospy.Time.now()
            self.goal_pose.pose = self.final_robot_pose
            self.goal_pub.publish(self.goal_pose)
            self.execute = False
            return

        #front_dist = msg.ranges[0]
        # front_dists = msg.ranges[:20]+ msg.ranges[340:]
        # front_dist = min(list(map(lambda x: inf_distance if x == float('inf') else x, front_dists)))
        size = len(msg.ranges)
        front_dists = msg.ranges[:half_of_scan_size]+ msg.ranges[(size - half_of_scan_size):]
        front_dist = min(list(map(lambda x: inf_distance if (x == float('inf') or x == 0.0
        ) else x, front_dists)))
        rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)

        if self.before_gas_value < self.gas_value:
            self.explore_state = 'front'

        if (front_dist < self.territory_radius):
            self.cmd_x = 0.0
            self.cmd_yaw = 0.5
            self.explore_state = 'front'
            rospy.loginfo_throttle(1.0, 'state: avoiding obstacle',)
        else:
            self.cmd_yaw = 0.0
            self.cmd_x = 0.0
            rospy.loginfo_throttle(1.0, 'state: %s', self.explore_state)
            self.explore()

        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_x
        cmd_msg.angular.z =self.cmd_yaw
        self.vel_pub.publish(cmd_msg)

if __name__ == "__main__":
    try:
        gas_scrutinize_action = gas_scrutinize()
    except rospy.ROSInterruptException: pass
