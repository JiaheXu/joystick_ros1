#!/usr/bin/env python3
import rospy
from mmpug_msgs.msg import ConvoyPositionArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
import time
import math

class ConvoyBotManager():
    def __init__(self, sys_id):
        self.system_id = sys_id

        # Subscribers and Publishers
        self.odometry_sub = rospy.Subscriber("/%s/integrated_to_global"%self.system_id, Odometry, self.odometryCallback)
        self.is_forward_sub = rospy.Subscriber("/%s/convoy_interface/is_forward"%self.system_id, Bool, self.isForwardCallback)

        # Variables
        self.odometry = None
        self.has_odom = False
        self.is_forward = Bool()
        self.is_forward.data = True
        self.has_forward = False

    def odometryCallback(self, msg):
        self.odometry = msg
        self.has_odom = True

    def isForwardCallback(self, msg):
        self.is_forward = msg
        self.has_forward = True


class ConvoyManager():
    def __init__(self):
        # Params
        self.update_hz = rospy.get_param("~update_hz", 50)
        self.system_id = rospy.get_param("~system_id", "rc1")
        self.init_frame = rospy.get_param("~init_frame", "rc1_sensor_init")

        # Subscribers and Publishers
        self.odom_sub = rospy.Subscriber("integrated_to_global", Odometry, self.odomCallback)
        self.convoy_switch_sub = rospy.Subscriber("convoy_interface/convoy_switch", Bool, self.convoySwitchCallback)
        self.convoy_pos_array_sub = rospy.Subscriber("convoy_interface/convoy_pos_array", ConvoyPositionArray, self.convoyPosCallback)
        self.convoy_reordered_sub = rospy.Subscriber("convoy_interface/convoy_reordered", Bool, self.convoyReoderedCallback)
        self.wp_sub = rospy.Subscriber("convoy_interface/convoy_waypoint", Pose, self.waypointsCallback)
        self.convoy_formup_sub = rospy.Subscriber("convoy_interface/convoy_formup_switch", Bool, self.convoyFormupCallback)
        self.target_speed_sub = rospy.Subscriber("executive/command/target_speed", Float32, self.targetSpeedCallback)
        self.freq_sub = rospy.Subscriber("/speed_ratio_freq" , Float32 , self.speedratiofreqCB)

        self.is_leader_pub = rospy.Publisher("convoy_interface/is_leader", Bool, queue_size=10) 
        self.following_pub = rospy.Publisher("convoy_interface/following", String, queue_size=10)
        self.following_odom_pub = rospy.Publisher("convoy_interface/following_odom", Odometry, queue_size=10)
        self.leader_odom_pub = rospy.Publisher("convoy_interface/leader_odom", Odometry, queue_size=10)
        self.speed_ratio_pub = rospy.Publisher("convoy_interface/speed_ratio", Float32, queue_size=10)
        self.is_forward_pub = rospy.Publisher("convoy_interface/is_forward", Bool, queue_size=10)
        self.in_convoy_pub = rospy.Publisher("convoy_interface/in_convoy", Bool, queue_size=10)

        # Variables
        self.convoy_switch = False
        self.following = self.system_id
        self.leading = self.system_id
        self.follow_flag = False
        self.lead_flag = False
        self.is_leader = Bool()
        self.is_leader.data = False
        self.following_dist = 0.0
        self.leading_dist = 0.0
        self.leading_speed = 0.0
        self.following_odom = None
        self.leader_odom = None
        self.leading_odom = None
        self.is_foll_odom = False
        self.is_lead_odom = False
        self.speed_ratio = Float32()
        self.speed_ratio.data = 1.0
        self.follow_speed_ratio = 1.0
        self.lead_speed_ratio = 1.0
        self.in_convoy_flag = Bool()
        self.in_convoy_flag.data = False
        self.in_comms_time = 0.0
        self.in_comms_cutoff_time = 3.0
        self.following_speed = 0.0
        self.current_speed = 0.0
        self.convoy_position = 0
        self.robots = {}
        self.robot_names = []
        self.convoy_pos_arr = ConvoyPositionArray()
        self.odom = None
        self.is_forward = Bool()
        self.is_forward.data = True
        self.forward_speed = 0.2        # Speed to be moving above for a change in direction
        self.forward_switch_time = 2    # Time moving in a direction before switching direction
        self.forward_time = 0
        self.forward_follow_flag = True
        self.convoy_reordered = False
        self.waypoint = None
        self.convoy_formup = False
        self.global_frame = "global"
        self.target_speed = 2.0
        self.ratio_pub_duration = 0.0
        self.ratio_check_time = time.time()

        # Tunable variables
        self.dist = 1.0
        self.speed_dist_mult = 0.5
        self.sensor_offset = 0.35

        self.max_follow_dist_2 = 2.0
        self.min_follow_dist_2 = 2.0
        self.near_slowdown_c_2 = 3.0
        self.near_kill_c_2 = 0.8
        self.far_slowdown_2 = 6.0
        self.far_kill_2 = 10.0

        self.max_follow_dist_4 = 2.5
        self.min_follow_dist_4 = 2.5
        self.near_slowdown_c_4 = 4.0
        self.near_kill_c_4 = 1.5
        self.far_slowdown_4 = 10.0
        self.far_kill_4 = 18.0
    
    def run(self):
        r = rospy.Rate(self.update_hz)

        while not rospy.is_shutdown():
            # Reset all flags
            self.resetVariables()

            # In convoy if called
            if self.convoy_switch:
                self.setConvoyPosition()
                self.setFollowing()

                # If following another vehicle
                if self.follow_flag:
                    self.getFollowingOdom()
                    self.getGapDist()
                    self.in_convoy_flag.data = True

                    if self.is_foll_odom:
                        self.leader_odom = self.following_odom
                        self.leader_odom_pub.publish(self.leader_odom)
                        self.shiftFollowingOdom()
                        self.following_dist = self.getBotDist(self.odom, self.following_odom)
                        self.follow_speed_ratio = self.followSpeedRatio()
                        self.following_pub.publish(self.following)
                        self.following_odom_pub.publish(self.following_odom)
                else:
                    self.follow_speed_ratio = 1.0
                
                # If leading another vehicle
                if self.lead_flag:
                    self.getLeadingOdom()
                    self.in_convoy_flag.data = True

                    if self.is_lead_odom and not self.convoy_formup:
                        self.leading_dist = self.getBotDist(self.odom, self.leading_odom)
                        self.leading_speed = self.leading_odom.twist.twist.linear.x
                        self.lead_speed_ratio = self.leadSpeedRatio()
                    elif self.convoy_formup:
                        self.lead_speed_ratio = 1.0
                else:
                    self.lead_speed_ratio = 1.0
 
                # Safety stop robots if comms drops
                if self.testComms():
                    self.speed_ratio.data = self.follow_speed_ratio * self.lead_speed_ratio
                    if self.lead_flag:
                        self.speed_ratio.data = min(abs(self.leading_speed) + 2.0, self.target_speed * self.speed_ratio.data) / self.target_speed
                else:
                    self.speed_ratio.data = 0.0
                
                if(time.time() - self.ratio_check_time >= self.ratio_pub_duration):
                    self.speed_ratio_pub.publish(self.speed_ratio)  
                    self.ratio_check_time = time.time()

            self.setForward()
            self.in_convoy_pub.publish(self.in_convoy_flag)
            self.is_leader_pub.publish(self.is_leader)
            r.sleep()
    
    def resetVariables(self):
        self.following = self.system_id
        self.leading = self.system_id
        self.follow_flag = False
        self.lead_flag = False
        self.convoy_position = 0
        self.in_convoy_flag.data = False
        self.is_leader.data = False
        self.is_foll_odom = False
        self.is_lead_odom = False

        slope = ((self.target_speed - 2)/2)
        self.max_follow_dist = self.interpolateLinear(self.max_follow_dist_2, self.max_follow_dist_4, slope)
        self.min_follow_dist = self.interpolateLinear(self.min_follow_dist_2, self.min_follow_dist_4, slope)
        self.near_slowdown_c = self.interpolateLinear(self.near_slowdown_c_2, self.near_slowdown_c_4, slope)
        self.near_kill_c = self.interpolateLinear(self.near_kill_c_2, self.near_kill_c_4, slope)
        self.far_slowdown = self.interpolateLinear(self.far_slowdown_2, self.far_slowdown_4, slope)
        self.far_kill = self.interpolateLinear(self.far_kill_2, self.far_kill_4, slope)

        if self.forward_follow_flag:
            self.near_slowdown = self.near_slowdown_c
            self.near_kill = self.near_kill_c
        else:
            self.near_slowdown = self.near_slowdown_c + self.sensor_offset
            self.near_kill = self.near_kill_c + self.sensor_offset
    
    def interpolateLinear(self, y1, y2, slope):
        temp = y1 + slope * (y2 - y1)
        return min(max(temp, y1), y2)

    
    def setForward(self):
        if self.odom is not None:
            # If convoy has been reordered, set forward conditions
            if self.convoy_reordered and self.convoy_position == 1 and self.waypoint is not None:
                pos = self.odom.pose.pose
                self.is_forward.data = self.findDirection(pos, self.waypoint)
                self.convoy_reordered = False
                self.waypoint = None
            elif self.convoy_reordered and self.convoy_position > 1 and self.is_foll_odom:
                pos = self.odom.pose.pose
                pos2 = self.following_odom.pose.pose
                self.is_forward.data = self.findDirection(pos, pos2)
                self.convoy_reordered = False
            else:
                # Moving forward
                if self.odom.twist.twist.linear.x > self.forward_speed:
                    # Flag and motion match, update time
                    if self.is_forward.data:
                        self.forward_time = time.time()
                    # Going from reverse to forward
                    else:
                        if time.time() - self.forward_time > self.forward_switch_time:
                            self.is_forward.data = True

                # Moving in reverse
                elif self.odom.twist.twist.linear.x < -self.forward_speed:
                    # Flag and motion match, update time
                    if not self.is_forward.data:
                        self.forward_time = time.time()
                    # Going from forward to reverse
                    else:
                        if time.time() - self.forward_time > self.forward_switch_time:
                            self.is_forward.data = False
                
                # Very slow motion, update time
                else:
                    self.forward_time = time.time()
            
            # Publish forward motion flag
            self.is_forward_pub.publish(self.is_forward)
    
    def findDirection(self, r1, r2):
        quaternion = [r1.orientation.x, r1.orientation.y, r1.orientation.z, r1.orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        yaw12 = math.atan2(r2.position.y - r1.position.y, r2.position.x - r1.position.x)
        ang = self.wrap2pi(yaw - yaw12)
        print("Angle for ", self.system_id, "is", ang)
        if abs(ang) < math.pi/2:
            return True
        return False

    def wrap2pi(self, yaw):
        if yaw > math.pi:
            yaw -= 2*math.pi
        if yaw <= -math.pi:
            yaw += 2*math.pi
        return yaw
    
    def setConvoyPosition(self):
        for position, convoy_arr in enumerate(self.convoy_pos_arr.convoy):
            if self.system_id == convoy_arr.system_id:
                self.convoy_position = convoy_arr.convoy_position
                if self.convoy_position == 1:
                    self.is_leader.data = True
                break
    
    def setFollowing(self):
        if self.convoy_position > 1:
            for position, convoy_arr in enumerate(self.convoy_pos_arr.convoy):
                if self.convoy_position - 1 == convoy_arr.convoy_position:
                    self.following = convoy_arr.system_id
                    self.follow_flag = True
                    break
        if self.convoy_position >= 1:
            for position, convoy_arr in enumerate(self.convoy_pos_arr.convoy):
                if self.convoy_position + 1 == convoy_arr.convoy_position:
                    self.leading = convoy_arr.system_id
                    self.lead_flag = True
                    break
    
    def testComms(self):
        if self.follow_flag or self.lead_flag:
            if self.is_foll_odom or self.is_lead_odom:
                self.in_comms_time = time.time()
            elif time.time() - self.in_comms_time > self.in_comms_cutoff_time:
                return False
        return True

    def getFollowingOdom(self):
        if self.following in self.robot_names:
            cr = self.robots[self.following]
            if cr.has_odom == True:
                self.following_odom = cr.odometry
                self.following_speed = cr.odometry.twist.twist.linear.x
                cr.has_odom = False
                self.is_foll_odom = True
            if cr.has_forward == True:
                self.forward_follow_flag = cr.is_forward.data

    def getLeadingOdom(self):
        if self.leading in self.robot_names:
            cr = self.robots[self.leading]
            if cr.has_odom == True:
                self.leading_odom = cr.odometry
                self.is_lead_odom = True
                cr.has_odom = False
    
    def getGapDist(self):
        self.dist = min((self.current_speed - self.following_speed)*self.speed_dist_mult + self.min_follow_dist, self.max_follow_dist)
    
    def shiftFollowingOdom(self):
        quaternion = [self.following_odom.pose.pose.orientation.x, self.following_odom.pose.pose.orientation.y, \
            self.following_odom.pose.pose.orientation.z, self.following_odom.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        if self.forward_follow_flag:
            self.following_odom.pose.pose.position.x += self.dist*(math.cos(yaw+math.pi))
            self.following_odom.pose.pose.position.y += self.dist*(math.sin(yaw+math.pi))
        else:
            self.following_odom.pose.pose.position.x -= self.dist*(math.cos(yaw+math.pi)) + self.sensor_offset
            self.following_odom.pose.pose.position.y -= self.dist*(math.sin(yaw+math.pi)) + self.sensor_offset
    
    def getBotDist(self, odom1, odom2):
        x1 = odom1.pose.pose.position.x
        y1 = odom1.pose.pose.position.y
        x2 = odom2.pose.pose.position.x
        y2 = odom2.pose.pose.position.y
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    def followSpeedRatio(self):
        if self.following_dist < self.near_kill:
            return 0.
        return min((self.following_dist - self.near_kill) / self.near_slowdown, 1.4)

    def leadSpeedRatio(self):
        if self.leading_dist > self.far_kill:
            return 0.
        return min((self.far_kill - self.leading_dist) / (self.far_kill - self.far_slowdown), 1.0)
    
    def convoySwitchCallback(self, convoy_switch):
        self.convoy_switch = convoy_switch.data
    
    def convoyPosCallback(self, convoy_position):
        self.convoy_pos_arr = convoy_position

        for i in range(0, len(convoy_position.convoy)):
            if convoy_position.convoy[i].system_id not in self.robot_names:
                syst_id = convoy_position.convoy[i].system_id
                self.robots[syst_id] = ConvoyBotManager(syst_id)
                self.robot_names.append(syst_id)
                self.robot_names.sort()
    
    def convoyReoderedCallback(self, msg):
        self.convoy_reordered = msg.data
    
    def odomCallback(self, msg):
        self.odom = msg
        self.current_speed = msg.twist.twist.linear.x
    
    def waypointsCallback(self, msg):
        self.waypoint = msg
    
    def convoyFormupCallback(self, msg):
        self.convoy_formup = msg.data
    
    def targetSpeedCallback(self, msg):
        self.target_speed = msg.data
    
    def speedratiofreqCB(self, msg):
        if (msg.data !=0):
            self.ratio_pub_duration = (1/msg.data)
        else:
            self.ratio_pub_duration = 100.0

if __name__ == "__main__":
    rospy.init_node("convoy_interface")
    tf_manager = ConvoyManager()
    tf_manager.run()