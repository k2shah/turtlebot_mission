#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray
from astar import AStar, StochOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Navigator:

    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)

        self.plan_resolution = 0.125
        self.plan_horizon = 10

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None

        self.nav_sp = None

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/turtlebot_mission/nav_goal_explore", Float32MultiArray, self.nav_sp_callback)

        self.pose_sp_pub = rospy.Publisher('/turtlebot_mission/position_goal', Float32MultiArray, queue_size=10)
        self.nav_path_pub = rospy.Publisher('/turtlebot_mission/path_goal', Path, queue_size=10)

    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution) * 2,
                                                  self.map_probs)

    def nav_sp_callback(self,msg):
        self.nav_sp = (msg.data[0],msg.data[1],msg.data[2])
        self.send_pose_sp()

    def buildPath(self, path, ref_tf='map'):
        path_msg = Path()
        path_msg.header.frame_id = ref_tf
        for i,state in enumerate(path):
            pose_st = PoseStamped()
            #package x,y
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            if i+1<len(path):
                theta= np.arctan2(path[i+1][1]-state[1], path[i+1][0]-state[0] ) #angle to next point 
            else:
                theta=self.nav_sp[2] #final pose
            #transform to quat
            quaternion=tf.transformations.quaternion_from_euler(0, 0, theta)
            #package theta
            pose_st.pose.orientation.x = quaternion[0]
            pose_st.pose.orientation.y = quaternion[1]
            pose_st.pose.orientation.z = quaternion[2]
            pose_st.pose.orientation.w = quaternion[3]
            #ref transform 
            pose_st.header.frame_id = ref_tf
            path_msg.poses.append(pose_st)
        return path_msg


    def send_pose_sp(self):

        try:
            (robot_translation,robot_rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_translation = (0,0,0)
            robot_rotation = (0,0,0,1)
            self.has_robot_location = False

        if self.occupancy and self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (int(round(robot_translation[0])), int(round(robot_translation[1])))
            x_goal = ((round(self.nav_sp[0]*self.plan_resolution))/self.plan_resolution, round(self.nav_sp[1]*self.plan_resolution)/self.plan_resolution)
            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            rospy.loginfo("Computing Navigation Plan")
            
            if astar.solve():
                rospy.loginfo("Navigation Success. Found %d waypoint path to (%6.3f, %6.3f)", len(astar.path), astar.path[-1][0], astar.path[-1][1])
                # a naive path follower we could use
                # pose_sp = (astar.path[1][0],astar.path[1][1],self.nav_sp[2])
                # msg = Float32MultiArray()
                # msg.data = pose_sp
                # self.pose_sp_pub.publish(msg)
                # astar.plot_path()
                
                path_msg=self.buildPath(astar.path)
                self.nav_path_pub.publish(path_msg)

            else:
                rospy.logwarn("Navigation Failure")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    nav = Navigator()
    nav.run()
