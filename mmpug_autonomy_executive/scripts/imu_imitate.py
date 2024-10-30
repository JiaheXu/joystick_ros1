#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import time

class ImuData():
    def __init__(self):
        self.imu_msg = Imu()
        self.fail_time = 1000.0
        self.start_time = 0.0

        self.update_hz = rospy.get_param("~update_hz", 20)

        self.imu_sub = rospy.Subscriber('imu/data', Imu,self.imu_callback)
        self.failslamsub = rospy.Subscriber('imu_imitate/slam_fail', Bool,self.failslamCB, queue_size=5)

        self.imu_pub = rospy.Publisher('imu/data', Imu,queue_size=10)


    def run(self):
        r = rospy.Rate(self.update_hz)

        while not rospy.is_shutdown():

            if time.time() - self.start_time < self.fail_time:
                if round(time.time() - self.start_time) % 2 == 0:
                    self.imu_msg.linear_acceleration.x += 15
                else:
                    self.imu_msg.linear_acceleration.x -= 15
                self.imu_msg.header.stamp = rospy.Time.now()
                self.imu_pub.publish(self.imu_msg)

            r.sleep()

    def imu_callback(self, msg):
        self.imu_msg = msg
    
    def failslamCB(self, msg):
        if msg.data:
            self.start_time = time.time()
            
if __name__=='__main__':
    rospy.init_node("imu_imitate")
    node = ImuData()
    node.run()