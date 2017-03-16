#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import tf
import numpy as np

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.getPose)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        #transform listner
        self.trans_listener = tf.TransformListener()


        #TUNNING PARAMS
        self.path_tresh= .1 #dist to target the next wp
        self.spin_gain = 1
        self.spin_rate =2

        # CHANGE TO MISSION WHEN JAMES IS DONE  
        rospy.Subscriber('/turtlebot_mission/path_goal', Path, self.updatePath)
        rospy.Subscriber('/turtlebot_mission/override', Float32MultiArray, self.override)


        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.x_g=0.0
        self.y_g=0.0
        self.th_g=0.0

        self.path=[]
        self.taskComplete=False
        #self.path=[[0.0, 1.0, np.pi/2], [2.0, 1.0, 0.0], [2.0, 0.0, -np.pi/2], [3.0, 0.0, 0.0]  ]  #test path


        #override
        self.cmdState = 0
        self.cmd_V = 0
        self.cmd_w = 0

        #todo: add inital angle alignment to reduce path arcs. 

    def updatePath(self, msg):
        rospy.loginfo("Path Updated")
        self.path=[ps.pose for ps in msg.poses] #list of pose obected, pre parased from PATH and POSE STAMMPED

    def override(self, msg):
        #get gets called when override is published 
        self.cmdState, self.cmd_V, self.cmd_w =msg.data

    def pathParse(self):
        pose=self.path.pop(0)
                      
        if len(self.path)==0:
             rospy.logwarn("Path Buffer Empty")

        #unpack and convert to euler
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        #return reduced pose
        return pose.position.x, pose.position.y, euler[2]



        print self.path

    def updateState(self):
        #gets state from transform tree
        try:
            (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rotation)
            #state
            self.x=translation[0]
            self.y=translation[1]
            self.th = euler[2]
            #print("\nstate from translation\n")
            #print(self.x, self.y, self.theta)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


        
    def get_ctrl_output(self):
        # use self.x self.y and self.theta to compute the right control input here
        # cmd_x_dot = 0.0 # forward velocity
        # cmd_theta_dot = 0.0

        def wrapToPi(a):
            b = a
            if a < -np.pi or a > np.pi:
                b = ((a+np.pi) % (2*np.pi)) - np.pi
            return b
        #get pose from map stuff
        self.updateState()


        #init cmd
        cmd = Twist()
        #unpack to local 
        x=self.x; y=self.y; th=self.th
        # print("state\n")
        # print(self.x, self.y, self.th)
        x_g=self.x_g; y_g=self.y_g; th_g=self.th_g #goal
        p=((x-x_g)**2+(y-y_g)**2)**(0.5) #distance to target    
        

        
        if  p<self.path_tresh: #get get point if close
            if len(self.path)==0 :
                cmd.linear.x=0 #shut speed down
                cmd.angular.z= self.spin_gain*(th_g-th) #P control to angle
                return cmd
            else:
                self.x_g, self.y_g, self.th_g= self.pathParse()
                rospy.loginfo("Waypoint Update: %f, %f, %f",          
                                                    self.x_g, self.y_g, self.th_g)

        #unpack pose
        x_g=self.x_g; y_g=self.y_g; th_g=self.th_g #goal

        #SPIN IN PLACE UNTILL ALIGNED
        # if np.abs(th_g-th) > .1:
        #     print("spinning\n")
        #     cmd.linear.x=0
        #     cmd.angular.z= self.spin_gain*(th_g-th) #P CONTROLLLLLLLLLLL
        #     return cmd

        ##START POSE CONTROLER######
        #ref angle
        b=np.arctan2(y_g-y, x_g-x)
        d=wrapToPi(b-th_g)
        a=wrapToPi(b-th)

        #control law
        
        k1=.8
        if p< self.path_tresh*2:
            k1=1 #keep from slowing towards the end
        k2=.8
        k3=.8

        V=k1*p*np.cos(a)
        om=k2*a+k1*np.sinc(a/np.pi)*np.cos(a)*(a+k3*d)
        #Define control inputs (V,om) - without saturation constraints
        # Apply saturation limits
        cmd_x_dot = np.sign(V)*min(0.5, np.abs(V))
        cmd_theta_dot = np.sign(om)*min(1, np.abs(om))

        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        #print(cmd)
        return cmd


    def override_output():
        cmd = Twist()
        cmd.linear.x = self.cmd_V
        cmd.angular.z = self.cmd_w
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            #cmd is a Twist
            if self.cmdState==0: #autonomous mode
                cmd = self.get_ctrl_output() 
            else:
                cmd=self.override_output()




            #### THIS MAKES IT MOVE
            self.pub.publish(cmd)
            ####

            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
