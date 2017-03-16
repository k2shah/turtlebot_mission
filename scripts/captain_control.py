#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np

class CaptianControl:

  def __init__(self):
    rospy.init_node('turtlebot_captain', anonymous=True)
    self.cmd_pub = rospy.Publisher('/turtlebot_mission/explore_command', String, queue_size=10)
    self.nav_goal_pub = rospy.Publisher('/turtlebot_mission/nav_goal', String, queue_size=10)
    self.err_state = -10**4
    self.cmd = ''

  def loop(self):
    send_cmd = False
    self.cmd = ''
    try:
      print('\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n')
      print('Hit me with it') # input() no good, cant deal w/ straight letters w/o quotes
      k_msg = raw_input() # output is a string
      comma_ind = k_msg.find(',')
      if(comma_ind == -1): # no comma - check if one of our commands
        if(len(k_msg)==1):
          # print 'I\'m hearing: %s\n' %k_msg  
          if(  'h'==k_msg):
            self.printCommandList()
          elif('e'==k_msg):
            self.cmd = 'MOTORS_ENABLED'
            send_cmd = True
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
            print('Enter amount to rotate (degrees)')
            ang = self.readSingleChar()
            if(ang != self.err_state):
              self.cmd = 'ROTATE_%.2f' %ang
              send_cmd = True
            else:
              send_cmd = False
          elif('x'==k_msg):
            print('Enter new desired x')
            x_goal = self.readSingleChar()
            if(x_goal != self.err_state):
              self.cmd = 'SET_X_%.2f' %x_goal
              send_cmd = True
            else:
              send_cmd = False
          elif('y'==k_msg):
            print('Enter new desired y')
            y_goal = self.readSingleChar()
            if(y_goal != self.err_state):
              self.cmd = 'SET_Y_%.2f' %y_goal
              send_cmd = True
            else:
              send_cmd = False
          elif('t'==k_msg):
            print('Enter new desired theta (degrees)')
            t_goal = self.readSingleChar()
            if(t_goal != self.err_state):
              self.cmd = 'SET_T_%.2f' %t_goal
              send_cmd = True
            else:
              send_cmd = False
          else:
            send_cmd = False
        else:
          send_cmd = False
      else: # has at least 1 comma
        x_s = k_msg[:comma_ind]
        y_s = k_msg[(comma_ind+1):]
        if( (',' in x_s) or (',' in y_s)): # too many commas
          send_cmd = False
        else:
          try:
            x = float(x_s); y = float(y_s)
            self.cmd = 'SET_XY_%.2f_%.2f' %(x,y)
            send_cmd = True
          except ValueError:
            send_cmd = False
    except NameError: 
      send_cmd = False

    if(send_cmd and (self.cmd != '') ):
      print 'SENDING: %s' %self.cmd
      self.cmd_pub.publish(self.cmd)
    else:
      print 'Try again'

  def readSingleChar(self):
    f = self.err_state
    c = raw_input()
    try:
      f = float(c)
    except ValueError: pass
    return f

  def printCommandList(self):
    print('Commands:\n\t(h) <==> display command list again')
    print('\t(e) <==> enable motors')
    print('\t(d) <==> disable motors')
    print('\t(a) <==> switch to explore mode (manual waypoints)')
    print('\t(z) <==> switch to exploit mode (autonomous waypoints)')
    print('\t(#,#) <==> set desired x,y coordinate')
    print('\t(s) <==> spin in full circle')
    print('\t(r) <==> rotate theta degrees (can be <0))')
    print('\t(x) <==> prompt for desired x coordinate')
    print('\t(y) <==> prompt for desired y coordinate')
    print('\t(t) <==> prompt for desired theta (in degrees)')

  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    self.printCommandList()
    while not rospy.is_shutdown():
      self.loop()
      rate.sleep()

if __name__ == '__main__':
  capt = CaptianControl()
  capt.run()