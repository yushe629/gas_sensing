#!/usr/bin/env python
from sysconfig import is_python_build
from turtle import position
import rospy
import smach
import smach_ros
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Bool

class RoughlySearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'continue', 'error'])
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.estimated_gas_map_sub = rospy.Subscriber("estimated_gas_map",OccupancyGrid,  self.callback)

        self.gas_visual_map = OccupancyGrid()
        self.gas_visual_map.header.frame_id = "map"
        self.gas_scale = rospy.get_param("~gas_visual_scale", 1.0)
        self.gas_offset = rospy.get_param("~gas_visual_offset", 0.0)

        # set params for get maximum position
        self.gas_visual_map.info.resolution = rospy.get_param("~gas_visual_map_resolution", 0.05) # 0.05 m/pixel
        origin = rospy.get_param("~gas_visual_map_origin", [-10.0, -10.0, 0.0])
        self.gas_visual_map.info.width = int(math.fabs(origin[0]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.height = int(math.fabs(origin[1]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.origin.position.x = origin[0]
        self.gas_visual_map.info.origin.position.y = origin[1]
        self.gas_visual_map.info.origin.position.z = origin[2]
        self.gas_visual_map.info.origin.orientation.w = 1
        self.gas_visual_map.data = [0] * self.gas_visual_map.info.width * self.gas_visual_map.info.height

        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_callback)

        self.completed = False
        self.is_moving = False
        self.is_get_goal = False
        self.is_published_goal = False
        
        self.is_error = False
        
        self.target = PoseStamped()
        # rospy.sleep(1.0)

    def move_base_callback(self, msg):
        if not self.is_published_goal:
            return
        if msg.status.status == 3:
            rospy.loginfo("%s", msg.status.text)
            self.completed = True
        else:
            rospy.loginfo("%s",msg.status.text)

    def callback(self, msg):
        if not self.is_get_goal:
            mapdata = msg.data
            mapindex = mapdata.index(max(mapdata))
            target_x_pixel = mapindex % self.gas_visual_map.info.width
            target_y_pixel = mapindex / self.gas_visual_map.info.width
            target_x = target_x_pixel * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.x
            target_y = target_y_pixel * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.y
            self.target.header.frame_id = "map"
            self.target.header.seq = self.target.header.seq + 1
            self.target.header.stamp = rospy.Time.now()
            self.target.pose.position.x = target_x
            self.target.pose.position.y = target_y
            self.target.pose.position.z = 0.0
            self.target.pose.orientation.x = 0.0
            self.target.pose.orientation.y = 0.0
            self.target.pose.orientation.z = 0.0
            self.target.pose.orientation.w = 1.0
            self.is_get_goal = True
        return

    def execute(self, userdata):
        if self.completed:
            return 'completed'
        if self.is_get_goal and not self.is_published_goal:
            rospy.loginfo("could get goal")
            self.goal_pub.publish(self.target)
            self.is_published_goal = True
            return 'continue'
        elif self.is_published_goal or not self.is_get_goal:
            return 'continue'
        else:
            return 'error'