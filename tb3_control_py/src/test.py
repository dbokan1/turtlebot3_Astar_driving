#!/usr/bin/env python


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1




class Controller:
    def __init__(self, path_array):
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.path_array= path_array
        self.path_node_id = 0
        self.state='stop'
	    #possible states: stop, adjusting, in_transit
        self.lin_vel=0
        self.ang_vel=0
    
    def callback(self, odom_msg):
        
        siny = 2.0 * (odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.z + odom_msg.pose.pose.orientation.x * odom_msg.pose.pose.orientation.y)
        cosy = 1.0 - 2.0 * (odom_msg.pose.pose.orientation.y * odom_msg.pose.pose.orientation.y + odom_msg.pose.pose.orientation.z * odom_msg.pose.pose.orientation.z) 
        tb3_pose = math.atan2(siny, cosy)
        
        goal=self.path_array[self.path_node_id]
        dist_to_goal=math.sqrt((goal[1]-odom_msg.pose.pose.position.y)**2 + (goal[0]-odom_msg.pose.pose.position.x)**2)
        if dist_to_goal<1e-2:
            self.path_node_id+=1
            goal=self.path_array[self.path_node_id]
            print('Moving to node ', goal)
        goal_pose=math.atan2( (goal[1]-odom_msg.pose.pose.position.y),(goal[0]-odom_msg.pose.pose.position.x))
        
        twist = Twist()
        rel_err=(goal_pose-tb3_pose)/abs(goal_pose)
        
        if dist_to_goal>1e-2:
            twist.linear.x = 0.1; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        if rel_err>0.1 and dist_to_goal>1e-2:
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.1
        elif rel_err<-0.1 and dist_to_goal>1e-2:
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -0.1
        else:
            twist.linear.x = 0.1; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        
        #print(goal_pose, tb3_pose)
        #print('-'*20)
        self.pub.publish(twist)
    		


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.pose.pose)
    
    

if __name__=="__main__":
    
    rospy.init_node('test_node')
    path=[[-1.26,-1.6],[-0.74,-1.58],[-0.74,-0.45]]
    controller = Controller(path)
    rospy.spin()
