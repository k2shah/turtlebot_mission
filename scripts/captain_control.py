#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np

class CaptianControl:

  def __init__(self):
    rospy.init_node('turtlebot_captain', anonymous=True)
    # topic with commands sent to the FSM (e.g. spin in place, disable motor, etc)
    self.cmd_pub = rospy.Publisher('/turtlebot_mission/fsm_command', String, queue_size=10)

    # topic where goal state for autonomous explore mode
    self.nav_goal_pub = rospy.Publisher('/turtlebot_mission/nav_goal', Float32MultiArray, queue_size=10)

    # topic to recieve messages from FSM to print (for debug/info)
    self.verbose_sub = rospy.Subscriber('/turtlebot_mission/verbose', String, self.verboseListener)

    self.verbose_message = 'test'
    self.err_state = -10**4
    self.cmd = ''
    self.my_nav_goal = np.zeros(3)*1.0

# This is the main loop of the node. Node will wait until user hits enter. Actions are taken for certain input. 
#     For valid inputs, see the printCommandList(self) function. Invalid inputs will be ignored. Messages are
#     only sent when the user hits enter, otherwise raw_input() call is blocking.
# 
# General logic: if no comma, must be single letter command (if not, ignore). If not a recognized letter, ignore.
#     if more than 2 commas, ignore. If 1 or 2 commas, parse into #,# or #,#,#. Try/Except to catch if any casts
#     from string to floats fail (if failure, ignore).
  def loop(self):
    send_cmd = False
    self.cmd = ''
    my_nav_goal = Float32MultiArray()
    my_nav_goal.data = self.my_nav_goal
    nav_goal_updated = False
    try:
      print('\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n')
      print('Hit me with it') # input() no good, cant deal w/ letters w/o quotes
      k_msg = raw_input() # output is type String
      comma_ind = k_msg.find(',')
      if(comma_ind == -1): # no comma - check if one of our commands
        if(len(k_msg)==1): 
          if(  'h' == k_msg):
            self.printCommandList()
          elif('v'==k_msg):
            print('\nverbose message: \n%s' %self.verbose_message)
            self.printNavGoal()
          elif('d'==k_msg):
            self.cmd = 'MOTORS_DISABLED'
            send_cmd = True
          elif('a'==k_msg):
            self.cmd = 'EXPLORE_MODE'
            send_cmd = True
          elif('z'==k_msg):
            self.cmd = 'EXPLOIT_MODE'
            send_cmd = True
          elif('s'==k_msg):
            self.cmd = 'SPIN'
            send_cmd = True
          elif('r'==k_msg):
            print('\nEnter amount to rotate (degrees)')
            ang = self.readSingleChar()
            if( (ang != self.err_state) and (ang>-360) and (ang < 360) ):
              self.cmd = 'ROTATE_%.2f' %ang
              send_cmd = True
            else:
              send_cmd = False
          elif('x'==k_msg):
            print('\nEnter new desired x')
            x_goal = self.readSingleChar()
            if(x_goal != self.err_state):
              my_nav_goal.data[0] = x_goal
              nav_goal_updated = True
          elif('y'==k_msg):
            print('\nEnter new desired y')
            y_goal = self.readSingleChar()
            if(y_goal != self.err_state):
              my_nav_goal.data[1] = y_goal
              nav_goal_updated = True
          elif('t'==k_msg):
            print('\nEnter new desired theta (degrees)')
            t_goal = self.readSingleChar()
            if(t_goal != self.err_state and (t_goal>0) and (t_goal < 360) ):
              my_nav_goal.data[2] = t_goal
              nav_goal_updated = True
          else:
            send_cmd = False
        else:
          send_cmd = False
      else: # has at least 1 comma
        if(k_msg.count(',')<=2): # 3 or more = too many commas
          x_s = k_msg[:comma_ind]
          y_s = k_msg[(comma_ind+1):]
          t_ind = y_s.find(',')
          try:
            t_goal = my_nav_goal.data[2]
            if(t_ind>=0): # This means we have a theta
              t_s = y_s[(t_ind+1):]
              y_s = k_msg[(comma_ind+1):(comma_ind+1+t_ind)]
              t_goal = float(t_s)
              if( (t_goal>0) and (t_goal < 360) ):
                x_goal = float(x_s); y_goal = float(y_s)
                my_nav_goal.data[0] = x_goal
                my_nav_goal.data[1] = y_goal
                my_nav_goal.data[2] = t_goal
                nav_goal_updated = True
            else:# This means we have just x,y
                x_goal = float(x_s); y_goal = float(y_s)
                my_nav_goal.data[0] = x_goal
                my_nav_goal.data[1] = y_goal
                nav_goal_updated = True
          except ValueError: pass
    except NameError: 
      send_cmd = False

    if(nav_goal_updated):
      self.nav_goal_pub.publish(my_nav_goal)
      self.my_nav_goal = my_nav_goal.data
      self.printNavGoal()

    if(send_cmd and (self.cmd != '') ):
      print('\nSENDING: %s' %self.cmd)
      self.cmd_pub.publish(self.cmd)

  def readSingleChar(self):
    f = self.err_state
    c = raw_input()
    try:
      f = float(c)
    except ValueError: pass
    return f

  def verboseListener(self, msg):
    self.verbose_message = msg.data

  def printNavGoal(self):
    print('\nnav_goal = [%.2f, %.2f, %.2f]' %(self.my_nav_goal[0],self.my_nav_goal[1],self.my_nav_goal[2]))

  def printCommandList(self):
    print('\nCommands:\n\t(h) <==> display command list again\n\
      \t(v) <==> print latest verbose message\n\
      \t(d) <==> disable motors\n\
      \t(a) <==> switch to explore mode (manual waypoints)\n\
      \t(z) <==> switch to exploit mode (autonomous waypoints)\n\
      \t(#,#) <==> set desired x,y coordinate\n\
      \t(#,#,#) <==> set desired x,y,theta coordinate (theta in degrees)\n\
      \t(s) <==> spin in full circle\n\
      \t(r) <==> rotate theta degrees (can be <0))\n\
      \t(x) <==> prompt for desired x coordinate\n\
      \t(y) <==> prompt for desired y coordinate\n\
      \t(t) <==> prompt for desired theta (in degrees)\n')

  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    self.printCommandList()
    while not rospy.is_shutdown():
      self.loop()
      rate.sleep()

if __name__ == '__main__':
  capt = CaptianControl()
  capt.run()