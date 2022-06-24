#!/usr/bin/env python
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
# inf_distance = 5.0
# the radius of the robot explore area
# territory_radius = 1.0  for aibot
# half_of_scan_size = 20
# the velocity of exploring and the type of explore state
# explore_vel = 0.3
# explore_time = 0.3
# explore_yaw_vel = 1.0
# explore_yaw_time = 0.3
# explore_state = ['front', 'back', 'turn', 'after_turn', 'explored']

class gas_scrutinize:
    def __init__(self):

        rospy.init_node("gas_scrutinize")

        # subscribers and publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.is_finish_search_sub = rospy.Subscriber("/is_finish_search", Bool, self.search_callback)
        self.scan_sub = message_filters.Subscriber("/scan", LaserScan)
        self.gas_value_sub = message_filters.Subscriber("/gas", Gas)
        self.robot_pose_sub = message_filters.Subscriber("/odom", Odometry)
        self.delay = 1/100. * 0.5
        self.mf = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.gas_value_sub, self.robot_pose_sub], 10, self.delay)
        self.mf.registerCallback(self.callback)

        # params
        self.territory_radius = rospy.get_param("~territory_radius", 0.6)
        self.timeout_sec = rospy.get_param("~timeout_sec", 20.0)
        self.inf_distance = rospy.get_param("~inf_distance", 5.0)
        # half counts of Lasascan.data for using
        self.half_of_scan_size = rospy.get_param("~half_of_scan_size", 30)
        self.explore_vel_x = rospy.get_param("~explore_vel_x", 0.3)
        self.explore_vel_yaw = rospy.get_param("~explore_vel_yaw", 0.3)
        self.explore_time_x = rospy.get_param("~explore_time_x", 1.0)
        self.explore_time_yaw = rospy.get_param("~explore_time_yaw", 0.3)

        self.execute = False
        self.start_time = 0
        self.max_gas_value = 0.0
        self.final_robot_pose = Pose()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.cmd_x = 0.0
        self.cmd_yaw = 0.0
        self.explore_state = 'front'
        self.iscorrect_path = False
        self.max_gas_value_time = rospy.get_time()
        self.before_gas_value = 0.0

        rospy.spin()

    def search_callback(self, msg):
        self.execute = msg.data
        self.start_time = rospy.get_time()

    # def gas_odom_callback(self, gas_msg, odom_msg):
    #     if not self.execute:
    #         return
    #     self.gas_value = gas_msg.gas.data
    #     self.before_gas_value = self.gas_value
    #     self.gas_value = gas_msg.gas.data
    #     if gas_msg.gas.data > self.max_gas_value:
    #         self.max_gas_value = gas_msg.gas.data
    #         self.final_robot_pose = odom_msg.pose.pose
    #         self.max_gas_value_time = rospy.get_time()

    def explore(self):
        if self.explore_state == 'front':
            self.cmd_x = self.explore_vel_x
            self.cmd_yaw = 0.0
            self.explore_state = 'back'
        elif self.explore_state == 'back':
            rospy.sleep(self.explore_time_x)
            self.cmd_x = -1.0*self.explore_vel_x
            self.cmd_yaw = 0.0
            self.explore_state = 'turn'
        elif self.explore_state == 'turn':
            rospy.sleep(self.explore_time_x)
            self.cmd_x = 0.0
            self.cmd_yaw = self.explore_vel_yaw
            self.explore_state = 'after_turn'
        elif self.explore_state == 'after_turn':
            rospy.sleep(self.explore_time_yaw)
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            self.explore_state = 'front'

    def callback(self, scan_msg, gas_msg, odom_msg):
        if not self.execute:
            return

        if self.start_time == 0:
            self.start_time = rospy.get_time()
            self.max_gas_value_time = rospy.get_time()
            return

        if gas_msg.gas.data > self.max_gas_value:
            self.max_gas_value = gas_msg.gas.data
            self.final_robot_pose = odom_msg.pose.pose
            self.max_gas_value_time = rospy.get_time()

        # Check Time Out
        last_sec = (rospy.get_time() - self.max_gas_value_time)
        # rospy.loginfo_throttle(1.0, "last_sec: %f", last_sec)
        if last_sec  > self.timeout_sec:
            rospy.logwarn_once("Robot discovered goal!")
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            self.goal_pose.header.seq = self.goal_pose.header.seq + 1
            self.goal_pose.header.stamp = rospy.Time.now()
            self.goal_pose.pose = self.final_robot_pose
            self.goal_pub.publish(self.goal_pose)
            self.execute = False
            return

        size = len(scan_msg.ranges)
        front_dists = scan_msg.ranges[:self.half_of_scan_size]+ scan_msg.ranges[(size - self.half_of_scan_size):]
        front_dist = min(list(map(lambda x: self.inf_distance if (x == float('inf') or x == 0.0
        ) else x, front_dists)))
        # rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)

        if self.before_gas_value < gas_msg.gas.data:
            self.explore_state = 'front'

        if (front_dist < self.territory_radius):
            # Avoiding obstacle
            self.cmd_x = 0.0
            self.cmd_yaw = 0.5
            self.explore_state = "front"
            rospy.loginfo_throttle(1.0, 'state: avoiding obstacle',)
        else:
            self.cmd_yaw = 0.0
            self.cmd_x = 0.0
            rospy.loginfo_throttle(1.0, 'state: %s', self.explore_state)
            # Explore gas origin
            self.explore()

        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_x
        cmd_msg.angular.z =self.cmd_yaw
        self.vel_pub.publish(cmd_msg)
        self.before_gas_value = gas_msg.gas.data

if __name__ == "__main__":
    try:
        gas_scrutinize_action = gas_scrutinize()
    except rospy.ROSInterruptException: pass
