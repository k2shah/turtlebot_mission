#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]

def wrapToPi(a):
    if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"
        rospy.Subscriber('/turtlebot_mission/fsm_command', String, self.fsm_cmd_callback)   # input from command line

        self.override_pub = rospy.Publisher('/turtlebot_mission/override', Float32MultiArray, queue_size=10)
        self.verbose_pub = rospy.Publisher('/turtlebot_mission/verbose', String, queue_size=10)
        
        self.trans_listener = tf.TransformListener() # to get pose information

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = 0.4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

        self.all_tag_numbers = range(7)

        # information for autonomously visiting tags in order specified by mission
        self.nav_goal_exploit_pub = rospy.Publisher('/turtlebot_mission/nav_goal_exploit', Float32MultiArray, queue_size=10)
        self.mission_goal_sub = rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)
        self.tag_visit_order = None 
        self.tag_dist_thresh = 0.4
        self.tag_index = -1

        # 0: initialization state
        # 1: human-directed exploration
        # 2: autonomous mission execution
        # 3: interrupt state to rotate desired angle
        # 4: robot disabled
        self.state = "init"

        # current command from CMDLINE as well as commanded angle (if applicable)
        self.curr_cmd = ""
        self.cmd_angle = None

        # pose information of robot
        self.pose = None
        self.start_angle = None # work around for rotation commands
        self.angle_tol = 3


    def mission_callback(self,data):
        self.tag_visit_order = data.data

    def rviz_goal_callback(self, msg):
        pose_to_xyth(msg.pose)    # example usage of the function pose_to_xyth (defined above)
        # this callback does nothing... yet!

    def fsm_cmd_callback(self, msg):
        self.curr_cmd = msg.data
        if self.curr_cmd == "SPIN":
            self.cmd_angle = 2*np.pi - (self.angle_tol*np.pi/180)
        elif self.curr_cmd[0:6] == "ROTATE":
            self.cmd_angle = float(msg.data[7:])

    def update_waypoints(self):
        for tag_number in self.all_tag_numbers:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.waypoint_locations[tag_number] = self.trans_listener.transformPose("/map", self.waypoint_offset)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            if tag_number in self.waypoint_locations:
                wp = self.waypoint_locations[tag_number].pose
                self.trans_broad.sendTransform((wp.position.x, wp.position.y, 0),
                                               (wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w),
                                               rospy.Time.now(),
                                               "waypoint_{0}".format(tag_number),
                                               "/map")

    def get_current_pose(self):
        try:
            (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.pose = [translation[0], translation[1], euler[2]]
            if self.pose[2] < 0:
                self.pose[2] += 2*np.pi
        except:
            pass

    def check_global_events(self):
        if self.curr_cmd == "MOTORS_DISABLED":
            self.state = "disabled"

        elif self.curr_cmd == "EXPLOIT_MODE":
            if self.tag_visit_order is None:
                rospy.logwarn('mission spec has not been updated!')
                self.curr_cmd = ""
                return

            for el in set(self.tag_visit_order):
                if el not in self.waypoint_locations.keys():
                    rospy.logwarn('not all tags have been found!')
                    self.curr_cmd = ""
                    return

            rospy.logwarn("----------STARTING AUTONOMOUS MODE----------")
            # switch to exploit topic in navigator/controller
            data = Float32MultiArray()
            data.data = [2,0,0]
            self.override_pub.publish(data)

            # reset to redo autonomous mode
            self.tag_index = -1

            self.curr_cmd = ""
            self.state = "exploit"

        elif self.curr_cmd == "EXPLORE_MODE":
            self.state = "explore" 


    def run(self):
        rate = rospy.Rate(10) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.update_waypoints()

            # check for high-priority events, e.g. disable motors
            self.check_global_events()

            # broadcast information
            self.get_current_pose()

            num_found = "-"
            if self.tag_visit_order is None:
                num_total = "-"
            else:
                num_total = len(set(self.tag_visit_order))
                num_found = 0
                for el in self.waypoint_locations.keys():
                    if el in self.tag_visit_order:
                        num_found += 1

            data = 'waypoints found: %s\nwaypoints progress: %s/%s\nFSM state: %s\ncurrent pose: %s' %(self.waypoint_locations.keys(),num_found,num_total,self.state,self.pose)
            self.verbose_pub.publish(data)

            # starting state
            if self.state == "init":
                self.state = "explore"

            # human directed exploration
            elif self.state == "explore":
                data = Float32MultiArray()
                data.data = [0, 0, 0] # revert to autonomous mode
                self.override_pub.publish(data)

                if self.curr_cmd[0:6] == "ROTATE" or self.curr_cmd == "SPIN" and self.pose is not None:
                    self.start_angle = self.pose[2]
                    self.state = "explore_rotate"

            elif self.state == "explore_rotate":
                if self.cmd_angle < 0:
                    data = Float32MultiArray()
                    data.data = [1, 0, -1] # spin clockwise
                else:
                    data = Float32MultiArray()
                    data.data = [1, 0, 1] # spin anti-clockwise

                self.override_pub.publish(data)

                # check if near desired angle
                location = self.start_angle+self.cmd_angle
                if location < 0:
                    location += 2*np.pi
                if location > 2*np.pi:
                    location -= 2*np.pi

                error = self.pose[2] - location                
                if abs(error) < (self.angle_tol*np.pi/180):
                    data = Float32MultiArray()
                    data.data = [0, 0, 0] # revert to autonomous mode
                    self.override_pub.publish(data)
                    self.state = "explore"
                    self.curr_cmd = ""

            # autonomous way-point following
            elif self.state == "exploit":

                if self.tag_index == -1:
                    self.tag_index += 1
                    wp = self.waypoint_locations[self.tag_visit_order[self.tag_index]]

                    rospy.logwarn("heading to tag %s",self.tag_visit_order[self.tag_index])
                    data = Float32MultiArray()
                    #data.data = [wp.pose.position.x, wp.pose.position.y, 0]
                    data.data = pose_to_xyth(wp.pose)
                    self.nav_goal_exploit_pub.publish(data)

                wpx, wpy, wpth = pose_to_xyth(wp.pose)
                dist = np.sqrt((self.pose[0]-wpx)**2 + (self.pose[1]-wpy)**2)

                #rospy.logwarn("current tag: %s, dist to tag: %s",self.tag_visit_order[self.tag_index],dist)
                if dist <= self.tag_dist_thresh:
                    self.tag_index += 1
                    
                    if self.tag_index >= len(self.tag_visit_order):
                        rospy.logwarn("mission complete!")   
                        self.state = "disabled"    
                        break   

                    wp = self.waypoint_locations[self.tag_visit_order[self.tag_index]]
                    rospy.logwarn("heading to tag %s",self.tag_visit_order[self.tag_index])
                    data = Float32MultiArray()
                    #data.data = [wp.pose.position.x, wp.pose.position.y, 0]
                    data.data = pose_to_xyth(wp.pose)
                    self.nav_goal_exploit_pub.publish(data)
 

            # disabled motors
            elif self.state == "disabled":

                # send robot stop commands
                data = Float32MultiArray()
                data.data = [1, 0, 0] # override mode
                self.override_pub.publish(data)


            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
