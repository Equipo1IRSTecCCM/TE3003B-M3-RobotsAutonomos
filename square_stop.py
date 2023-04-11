#! /usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from matplotlib import pyplot as plt

pos = Twist()
break_v = 0
turn_b = 0

def callbackOdom(msg):
    global pos
    pos.linear.x = msg.pose.pose.position.x
    pos.linear.y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_,_,pos.angular.z) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def smallest_angle_diff(t,s):
    a = t - s
    a -= 2*np.pi if a > np.pi else -2*np.pi if a < -np.pi else 0
    return a

def callbackScan(msg):
    global break_v
    global turn_b
    scan = np.array(msg.ranges)
    ang = msg.angle_min - np.pi
    andf = msg.angle_max - np.pi
    inc = msg.angle_increment
    angles = np.arange(ang,andf+inc,inc)
    angles = angles[np.r_[-60:60]]
    scan_f = scan[np.r_[-60:60]]
    angles = angles[~np.isnan(scan_f)]
    scan_f = scan_f[~np.isnan(scan_f)]
    angles = angles[np.isfinite(scan_f)]
    scan_f = scan_f[np.isfinite(scan_f)]
    # Máximo alcanze 0.12 - recomendada 0.15
    if scan_f[scan_f < 0.35].shape[0] > 0:
        break_v = (scan_f.min() - 0.23)/scan_f.min()
        try:
            angle = smallest_angle_diff(angles[scan_f == scan_f.min()],np.pi)
            turn_b = angle + (4*np.pi)/9 if angle < 0 else angle - (4*np.pi)/9
        except:
            turn_b = 0
    else:
        break_v = 1
        turn_b = 0

def square():
    global pos
    global break_v
    global turn_b
    vel = Twist()

    rospy.init_node('square_scan_stop')
    
    raten = 10

    rate = rospy.Rate(raten)
    odom = rospy.Subscriber('/odom',Odometry,callbackOdom)
    scanS = rospy.Subscriber('/scan',LaserScan,callbackScan)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    
    kpl = 1
    kpa = 2
    rospy.sleep(1)
    thrd = 0.01
    thra = 0.01
    puntos = [[0,1],[1,1],[1,0],[0,0]]

    for goal in puntos:
        ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
        while abs(smallest_angle_diff(ang,pos.angular.z)) > thra:
            ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
            vel.angular.z = smallest_angle_diff(ang,pos.angular.z)*kpa
            vel.angular.z = vel.angular.z if abs(vel.angular.z) <= 2.84 else 2.84*np.sign(vel.angular.z)
            pub.publish(vel)
            rate.sleep()
        vel.angular.z = 0
        pub.publish(vel)
        rate.sleep()
        pub.publish(vel)
        rate.sleep()
        dis = np.sqrt((pos.linear.x - goal[0])**2 + (pos.linear.y - goal[1])**2)
        while dis > thrd:
            ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
            vel.angular.z = smallest_angle_diff(ang,pos.angular.z)*kpa if turn_b == 0 else turn_b*kpa*(1-break_v) if break_v >= 0 else 0
            vel.angular.z = vel.angular.z if abs(vel.angular.z) <= 2.84 else 2.84*np.sign(vel.angular.z)
            dis = np.sqrt((pos.linear.x - goal[0])**2 + (pos.linear.y - goal[1])**2)
            vel.linear.x = dis*kpl*(break_v)
            vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.22 else 0.22*np.sign(vel.linear.x)*abs(break_v)
            pub.publish(vel)
            rate.sleep()
        vel.angular.z = 0
        vel.linear.x = 0
        pub.publish(vel)
        rate.sleep()
        pub.publish(vel)
        rate.sleep()        

if __name__ == '__main__':
    try:
        square()
    except rospy.ROSInterruptException:
        pass