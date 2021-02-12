#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = -2
goal.y = 1

goal_x = [-1, -1, -6.5, ]#-6.5]
goal_y = [ 1,  4,    4, ]#   2]

distance_treshold = 0.05
linear_speed = 0.2
angle_treshold = 0.1
angular_speed = 0.2

i = 0
while not rospy.is_shutdown():
    inc_x = goal_x[i] - x
    inc_y = goal_y[i] - y

    distance_to_goal = math.sqrt(inc_x * inc_x + inc_y * inc_y)
    angle_to_goal = math.atan2(inc_y, inc_x) - theta
    if angle_to_goal > math.pi:
        angle_to_goal = angle_to_goal - 2 * math.pi
    elif angle_to_goal < -math.pi:
        angle_to_goal = angle_to_goal + 2 * math.pi
    # print("distance: {}, angle: {}".format(distance_to_goal, angle_to_goal))

    if distance_to_goal < distance_treshold:
        print("goal {} reached".format(i))
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        i = i + 1
    else:
        if abs(angle_to_goal) > angle_treshold:
            speed.linear.x = 0.0
            if angle_to_goal > angle_treshold:
                speed.angular.z = angular_speed
            else:
                speed.angular.z = -angular_speed
        else:
            speed.angular.z = 0.0
            r.sleep()
            speed.linear.x = linear_speed

    pub.publish(speed)
    r.sleep()
