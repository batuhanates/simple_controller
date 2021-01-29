#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sqrt

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

while not rospy.is_shutdown():
    inc_x = goal_x - x
    inc_y = goal.y - y

    distance_to_goal = sqrt(inc_x * inc_x + inc_y * inc_y)
    if distance_to_goal < 0.05:
        print("goal reached")
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        while True:
            r.sleep()

    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.2:
        speed.linear.x = 0.0
        if angle_to_goal > 0:
            speed.angular.z = 0.3
        else:
            speed.angular.z = -0.3
    else:
        speed.angular.z = 0.0
        r.sleep()
        speed.linear.x = 0.2


    pub.publish(speed)
    r.sleep()
