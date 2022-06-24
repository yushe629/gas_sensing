#!/usr/bin/env python
import imp
import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from turtlebot3_explore.msg import PositionAndGas
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
from turtlebot3_explore.msg import Gas

# if scan_val is Inf, inf_distance is assigned.
inf_distance = 5.0
# the radius of the robot explore area
territory_radius = 0.6
# the velocity of exploring and the type of explore state
explore_vel = 0.3
explore_time = 0.3
explore_yaw_vel = 1.0
explore_yaw_time = 0.3
explore_state = ['front', 'back', 'turn', 'after_turn', 'explored']
limit_time = 30.0

class gas_explore:
    def __init__(self):

        rospy.init_node("gas_explore")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.gas_map_pub= rospy.Publisher("/estimated_gas_map", OccupancyGrid, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)

        self.gas_value_sub = message_filters.Subscriber("/gas", Gas)
        self.robot_pose_sub = message_filters.Subscriber("/odom", Odometry)
        self.delay = 1/100. * 0.5
        self.mf = message_filters.ApproximateTimeSynchronizer([self.gas_value_sub, self.robot_pose_sub], 10, self.delay)
        self.mf.registerCallback(self.gas_odom_callback)
        
        self.before_gas_value = 0.0
        self.gas_value = 0.0
        self.max_gas_value = 0.0
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.robot_pose = None
        self.cmd_x = 0.0
        self.cmd_yaw = 0.0
        self.explore_state = 'front'
        self.iscorrect_path = True
        self.max_gas_value_time = rospy.Time()
        self.begin_time = rospy.Time.now()
        self.max_gas_value_pose = None

        # gas map for exploreation rule
        self.gas_map_array = np.empty((0,3), np.float32)
        # gap map for visualization
        self.gas_visual_map = OccupancyGrid()
        self.gas_visual_map.header.frame_id = "map"
        self.gas_scale = rospy.get_param("~gas_visual_scale", 1.0)
        self.gas_offset = rospy.get_param("~gas_visual_offset", 0.0)
        self.gas_visual_map.info.resolution = rospy.get_param("~gas_visual_map_resolution", 0.05) # 0.05 m/pixel
        origin = rospy.get_param("~gas_visual_map_origin", [-10.0, -10.0, 0.0])
        print(origin)
        self.gas_visual_map.info.width = int(math.fabs(origin[0]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.height = int(math.fabs(origin[1]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.origin.position.x = origin[0]
        self.gas_visual_map.info.origin.position.y = origin[1]
        self.gas_visual_map.info.origin.position.z = origin[2]
        self.gas_visual_map.info.origin.orientation.w = 1
        self.gas_visual_map.data = [0] * self.gas_visual_map.info.width * self.gas_visual_map.info.height
        self.gas_map_pub_time = rospy.get_time()

        rospy.spin()
        

    def time_now_in_node(self):
        t = rospy.Time.now()
        return t - self.begin_time

    def gas_odom_callback(self, gas_msg, odom_msg):
        # if not self.execute:
        #     return
        self.gas_value = gas_msg.gas.data
        if gas_msg.gas.data > self.max_gas_value:
            self.max_gas_value = gas_msg.gas.data
            self.final_robot_pose = odom_msg.pose.pose
            self.max_gas_value_time = rospy.get_time()

    
    # def odom_callback(self,msg):
    #     # set Odometry.pose.pose
    #     self.robot_pose = msg.pose.pose

    # def gas_callback(self, msg):
    #     self.before_gas_value = self.gas_value
    #     self.gas_value = msg.data
    #     # Add gas map date to gas_map_array
    #     if self.robot_pose != None:
    #         self.gas_map_array = np.append(self.gas_map_array, [[self.robot_pose.position.x, self.robot_pose.position.y, msg.data]], axis=0)

            # visualize the gas map
        #     pixel_x = int((self.robot_pose.position.x - self.gas_visual_map.info.origin.position.x) / self.gas_visual_map.info.resolution)
        #     pixel_y = int((self.robot_pose.position.y - self.gas_visual_map.info.origin.position.y) / self.gas_visual_map.info.resolution)
        #     self.gas_visual_map.data[pixel_y * self.gas_visual_map.info.width + pixel_x] = int(msg.data * self.gas_scale + self.gas_offset)

        #     if rospy.get_time() - self.gas_map_pub_time > 1.0: # publish in 1 Hz
        #         self.gas_map_pub.publish(self.gas_visual_map)
        #         self.gas_map_pub_time= rospy.get_time()

        # if msg.data > self.max_gas_value:
        #     self.max_gas_value = msg.data
        #     self.max_gas_value_time = self.time_now_in_node()
        #     self.max_gas_value_pose = self.robot_pose

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

        if self.explore_state == "explored":
            return

        # rospy.loginfo_throttle(1.0, "laser number: %d", len(msg.ranges))
        last_sec = (self.time_now_in_node().to_sec() - self.max_gas_value_time.to_sec())
        rospy.loginfo_throttle(1.0,"last_time: %f",last_sec)
        if last_sec  > limit_time:
            rospy.logwarn_once("Robot discovered goal!")
            self.explore_state = "explored"
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            self.goal_pose.header.seq = self.goal_pose.header.seq + 1
            self.goal_pose.header.stamp = rospy.Time.now()
            self.goal_pose.pose = self.robot_pose
            self.goal_pub.publish(self.goal_pose)
            return

        #front_dist = msg.ranges[0]
        front_dists = msg.ranges[:20]+ msg.ranges[340:]
        front_dist = min(list(map(lambda x: inf_distance if x == float('inf') else x, front_dists)))
        rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)
        if self.before_gas_value < self.gas_value:
            self.explore_state = 'front'

        # 180 (back):  msg.ranges[len(msg.ranges)/2]

        if (front_dist < territory_radius):
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
        # rospy.loginfo_throttle(1.0, "value: %s", str(self.gas_value))
        # rospy.loginfo_throttle(1.0, "before_gas_value: %s", str(self.before_gas_value))
        # rospy.loginfo_throttle(1.0, "maxvalue: %s", str(self.max_gas_value))
        # rospy.loginfo_throttle(1.0, "x: %f", self.cmd_x)
        # rospy.loginfo_throttle(1.0, "yaw: %f", self.cmd_yaw)
        # rospy.loginfo_throttle(1.0, "array: %s", self.gas_map_array)
        self.vel_pub.publish(cmd_msg)

if __name__ == "__main__":
    try:
        gas_explore_action = gas_explore()
    except rospy.ROSInterruptException: pass
