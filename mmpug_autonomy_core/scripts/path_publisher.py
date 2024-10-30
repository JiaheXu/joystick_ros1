#!/usr/bin/env python3
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mmpug_msgs.msg import DirectionalInput, GoalInput, PathVelocity
from geometry_msgs.msg import PointStamped, Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String, Float32
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from system_stats.msg import Float32Stamped
import time
import math
import string
    
class ExecutiveNode():
    def __init__(self):
        self.path_pub = rospy.Publisher("/planner/path", Path, queue_size=1)
        self.path_vel_pub = rospy.Publisher("/controller/path_velocity", PathVelocity, queue_size=1)
        self.file = "path.txt"
        self.points = False
        self.point = None
        
    def run(self):
        r = rospy.Rate(100)
        points_init = False
        last_points = time.time()

        xyz = []
        path = Path()
        i = 0
        with open(self.file, "r") as f:
            text = f.read()
            lines = text.split('\n')
            for l in lines:
                if l == "":
                    continue
                nums = l.split(",")
                xyz.append((float(nums[0]), float(nums[1]), float(nums[2])))
                cur_pt = PoseStamped()
                cur_pt.pose.orientation.w = 1.
                cur_pt.pose.position.x = float(nums[0])
                cur_pt.pose.position.y = float(nums[1])
                cur_pt.pose.position.z = float(nums[2])
                path.poses.append(cur_pt)
                cur_pt.header.frame_id = "sensor_init"
                i += 1
                # if i > 20:
                #     break

        path.header.frame_id = "sensor_init"
        path_vel = PathVelocity()
        path_vel.path = path
        path_vel.header.frame_id = "sensor_init"
        path_vel.target_velocity = 4.0
        # while True:
        time.sleep(1)
        self.path_pub.publish(path)
        self.path_vel_pub.publish(path_vel)
        time.sleep(2)
        exit(1)


if __name__ == "__main__":
    rospy.init_node("executive_node")
    node = ExecutiveNode()
    node.run()
