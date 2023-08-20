#!/usr/bin/env python


import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1




class Controller:
  def __init__(self, path_array):
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.sub = rospy.Subscriber("odom", Odometry, self.callback)
    self.path_array= path_array
    self.path_node_id = 0
    self.lin_vel=0.2
    self.ang_vel=0.2
    self.path_taken=[]
    
  def callback(self, odom_msg):
    # Racunanje poze orijentacije turtlebota kao ugao u odnosu na x osu
    siny = 2.0 * (odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.z + odom_msg.pose.pose.orientation.x * odom_msg.pose.pose.orientation.y)
    cosy = 1.0 - 2.0 * (odom_msg.pose.pose.orientation.y * odom_msg.pose.pose.orientation.y + odom_msg.pose.pose.orientation.z * odom_msg.pose.pose.orientation.z) 
    tb3_pose = math.atan2(siny, cosy)
    
    eps=1e-1

    goal=self.path_array[self.path_node_id]
    dist_to_goal=math.sqrt((goal[1]-odom_msg.pose.pose.position.y)**2 + (goal[0]-odom_msg.pose.pose.position.x)**2)
    self.path_taken.append([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
    
    # Priblizavanje bota eps okolini ciljne tacke, pomjeranje ciljne tacke na narednu
    if dist_to_goal<eps:
        self.path_node_id+=1
        if self.path_node_id>=len(self.path_array):
            numpy.save('/home/davorb/putanja.npy', numpy.array(self.path_taken))
        goal=self.path_array[self.path_node_id]
        print('Moving to node ', goal)
    
    # Racunanje relativnog ugla izmedju pozicije bota i ciljne tacke kao ugao u odnosu na x osu
    goal_pose=math.atan2( (goal[1]-odom_msg.pose.pose.position.y),(goal[0]-odom_msg.pose.pose.position.x))
    
    
    # Racunanje da li smjer orijentacije bota kao pravac upada u eps zonu oko ciljne tacke- ugaoni eps
    k=math.tan(tb3_pose)
    xt=odom_msg.pose.pose.position.x; yt=odom_msg.pose.pose.position.y
    xp=goal[0]; yp=goal[1]
    point_line_dist=abs(k*xp - yp + yt -k*xt)/math.sqrt(k**2+1)


    twist = Twist()
    twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0

    # Okretanje u pravcu ciljne tacke ili kretanje naprijed prema njoj
    if point_line_dist>eps and goal_pose-tb3_pose>0:
        twist.angular.z = self.ang_vel; twist.linear.x = 0.0
    elif point_line_dist>eps and goal_pose-tb3_pose<0:
        twist.angular.z = -self.ang_vel; twist.linear.x = 0.0
    else:
        twist.linear.x = self.lin_vel; twist.angular.z = 0.0


    self.pub.publish(twist)

    
    

if __name__=="__main__":
    
  rospy.init_node('test_node')
  # Slanje putanje dobivena od A*
  path=[(-1.95, -0.45), (-1.90, -0.45), (-1.75, -0.3), (-0.95, -0.3), (-0.8, -0.15), (-0.8, 0.05), (-0.05, 0.8), (0.2, 0.8), (0.7, 1.3), (0.7, 1.4), (0.75, 1.45), (1.4, 1.450), (1.45, 1.5), (1.5, 1.5)]

  controller = Controller(path)
  rospy.spin()

