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
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

        self.all_tag_numbers = range(7)

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

    def rviz_goal_callback(self, msg):
        pose_to_xyth(msg.pose)    # example usage of the function pose_to_xyth (defined above)
        # this callback does nothing... yet!

    def fsm_cmd_callback(self, msg):
        self.curr_cmd = msg.data
        if self.curr_cmd == "SPIN" or self.curr_cmd[0:6] == "ROTATE":
            self.cmd_angle = wrapToPi( float(msg.data[7:]) )

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
            #self.pose[2] = wrapToPi(self.pose[2])
        except:
            pass

    def check_global_events(self):
        if self.curr_cmd == "MOTORS_DISABLED":
            self.state = "disabled"

        elif self.curr_cmd == "EXPLOIT_MODE":
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
            data = 'waypoint_locations: %s\nFSM state: %s\ncurrent pose: %s' %(self.waypoint_locations,self.state,self.pose)
            self.verbose_pub.publish(data)

            # starting state
            if self.state == "init":
                self.state = "explore"

            # human directed exploration
            elif self.state == "explore":
                data = Float32MultiArray()
                data.data = [0, 0, 0] # revert to autonomous mode
                self.override_pub.publish(data)

                if self.curr_cmd[0:6] == "ROTATE":
                    self.start_angle = self.pose[2]
                    self.state = "explore_rotate"

            elif self.state == "explore_rotate":
                if self.cmd_angle < 0:
                    data = Float32MultiArray()
                    data.data = [1, 0, -0.5] # spin clockwise
                else:
                    data = Float32MultiArray()
                    data.data = [1, 0, 0.5] # spin anti-clockwise

                self.override_pub.publish(data)

                # check if near desired angle
                if abs(self.pose[2] - (self.start_angle+self.cmd_angle)) < 5:
                    data = Float32MultiArray()
                    data.data = [0, 0, 0] # revert to autonomous mode
                    self.override_pub.publish(data)
                    self.state = "explore"

            # autonomous way-point following
            elif self.state == "exploit":
                # not sure how to implement this
                pass                

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
