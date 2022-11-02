#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32

class GasGlobalMapping:
    def __init__(self):

        rospy.init_node("gas_global_mapping")

        # gas source coordinate
        self.gas_origin = rospy.get_param("~gas_origin", [2.0,0.5,0.0])
        self.gas_origin = np.array(self.gas_origin)
        # setting constant
        # self.alpha = rospy.get_param("~alpha", 1.0)
        self.max_val = rospy.get_param("~gap_max_val", 100)
        # define publisher and subscriber
        self.gas_map_pub = rospy.Publisher("/true_gas_map", OccupancyGrid, queue_size=10)
        # self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.callback)
        # setting rosTime
        self.start_time = 0
        self.time_rate_off = 1.0
        self.time_rate = 1.0
        self.dist_rate = 10.0

        # gas map for visualization
        self.gas_visual_map = OccupancyGrid()
        self.gas_visual_map.header.frame_id = "map"
        self.gas_scale = rospy.get_param("~gas_visual_scale", 1.0)
        self.gas_offset = rospy.get_param("~gas_visual_offset", 10.0)
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

        while not rospy.is_shutdown():
            self.map_gen()
            self.gas_map_pub.publish(self.gas_visual_map)
        rospy.spin()

        

        
    def calc_gas_value(self, pos):
        if self.start_time == 0:
            self.start_time = rospy.get_time()
            return
        # pos is 3d_array
        dist = np.linalg.norm(self.gas_origin - pos)
        del_time = rospy.get_time() - self.start_time
        # val = self.max_val*math.exp(-dist**2)*math.exp(-del_time)
        # val = self.max_val*math.exp(-dist**2) * (math.exp(- self.time_rate * (rospy.Time.now() - self.start_time).to_sec()) + self.time_rate_off)
        val = self.max_val*math.exp(-dist**2)*math.exp(-self.time_rate/(del_time + self.time_rate_off))
        return val

    def map_gen(self):
        if self.start_time == 0:
            self.start_time = rospy.get_time()
            return
        width = self.gas_visual_map.info.width
        height = self.gas_visual_map.info.height
        for i in range(width*height):
            x = i % width * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.x
            y = i / width * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.y
            half = self.gas_visual_map.info.resolution / 2
            value = self.calc_gas_value([x + half, y + half, 0])
            self.gas_visual_map.data[i] = int(value*self.gas_scale + self.gas_offset)

    # def callback(self, msg):
    #     self.gas_map_pub.publish(self.gas_visual_map)

if __name__ == "__main__":
    try:
        gas_global_mapping = GasGlobalMapping()

    except rospy.ROSInterruptException: pass
