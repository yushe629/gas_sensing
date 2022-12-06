#!/usr/bin/env python
from xml.etree.ElementTree import ProcessingInstruction
import rospy
import math
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler

PI = math.pi

#define Patrol State
class GasPatrolState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'completed', 'error'])
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
        # self.relay_points = rospy.get_param("~relay_points")
        # self.relay_points = [[2.0, -0.5, 0, PI/2.0], [2.0, 0.5,0, PI], [-2.0, 0.5, 0, 3*PI/2.0], [-2.0, -0.5, 0, 0]]
        # set debug demo points
        self.relay_points = [[0.5, 0.5, 0, 3.14], [-2.0, -0.5, 0, 0]]
        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.callback)
        self.is_moving = False
        self.is_completed = False
        self.nth_point = 0
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        # publish topic for next node
        rospy.sleep(1.0)

    def callback(self, msg):
        if msg.status.status == 3:
            rospy.loginfo("%s", msg.status.text)
            # wait 3 sec for more sensing
            rospy.sleep(3.0)
            self.is_moving = False
            if self.nth_point == len(self.relay_points):
                self.is_completed = True
        else:
            rospy.loginfo("%s",msg.status.text)

    def calc_plan(self, point):
        self.goal.header.seq = self.goal.header.seq + 1
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = point[0]
        self.goal.pose.position.y = point[1]
        self.goal.pose.position.z = point[2]
        q = quaternion_from_euler(0,0,point[3])
        self.goal.pose.orientation.x = q[0]
        self.goal.pose.orientation.y = q[1]
        self.goal.pose.orientation.z = q[2]
        self.goal.pose.orientation.w = q[3]

    def execute(self, userdata):
        if self.is_moving:
            return 'continue'
        elif self.is_completed:
            return 'completed'
        elif self.nth_point < len(self.relay_points):
            self.calc_plan(self.relay_points[self.nth_point])
            self.nth_point = self.nth_point+1
            self.goal_pub.publish(self.goal)
            self.is_moving = True
            return 'continue'
        else:
            self.counter += 1
            return 'error'