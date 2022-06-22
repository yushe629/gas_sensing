#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, UInt16
from turtlebot3_explore.msg import Gas

class get_co2:
    def __init__(self):
        
        rospy.init_node("get_co2")
        self.gas_pub = rospy.Publisher("/gas", Gas, queue_size=10)
        self.eco2_sub = rospy.Subscriber("/eco2", UInt16, self.callback)
        self.norm = 10.0
        rospy.spin()
        
    def callback(self, msg):
        gas_msg = Gas()
        gas_msg.header.stamp = rospy.Time.now()
        gas_msg.gas.data = float(msg.data)/self.norm
        self.gas_pub.publish(gas_msg)

if __name__ == "__main__":
    try:
        get_co2_action = get_co2()
    except rospy.ROSInterruptException: pass
