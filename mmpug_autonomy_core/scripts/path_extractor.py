#!/usr/bin/env python3
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mmpug_msgs.msg import DirectionalInput, GoalInput
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from system_stats.msg import Float32Stamped
import time
import math
import string
    
class ExecutiveNode():
    def __init__(self):
        self.odometry_sub = rospy.Subscriber("/integrated_to_init", Odometry, self.odometryCB)
        self.file = "path.txt"
        self.points = False
        self.point = None
        
    def run(self):
        r = rospy.Rate(100)
        points_init = False
        last_points = time.time()

        with open(self.file, "w+") as f:
            while not rospy.is_shutdown():
                if self.points:
                    last_points = time.time()
                    self.points = False
                    points_init = True
                    f.write("{},{},{}\n".format(self.point.pose.pose.position.x, self.point.pose.pose.position.y, self.point.pose.pose.position.z))
                elif points_init:
                    if time.time() - last_points > 10.0:
                        rospy.loginfo("No new points in 10 seconds. Dieing.")
                        exit(0)
                else:
                    rospy.loginfo_throttle(2, "Waiting for poitns...")
                r.sleep()

    ####
    # Callbacks!
    ####
    def odometryCB(self, msg):
        self.points = True
        self.point = msg


if __name__ == "__main__":
    rospy.init_node("executive_node")
    node = ExecutiveNode()
    node.run()
