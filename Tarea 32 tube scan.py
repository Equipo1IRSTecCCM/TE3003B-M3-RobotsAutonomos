#! /usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN

pos = Twist()
db = DBSCAN(eps=0.25, min_samples=2)
centers = []
rospy.init_node('scan_detect_fine')
rate = rospy.Rate(2)
pub = rospy.Publisher('/dopt_points',PointStamped,queue_size=10)

def callbackOdom(msg):
    global pos
    pos.linear.x = msg.pose.pose.position.x
    pos.linear.y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_,_,pos.angular.z) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def dis(src, goal):
    return np.sqrt((goal[0]-src[0])**2 + (goal[1]-src[1])**2)

def callbackScan(msg):
    global pos
    global db
    global centers
    global pub

    scan = np.array(msg.ranges)

    ang = msg.angle_min + pos.angular.z
    andf = msg.angle_max + pos.angular.z
    inc = msg.angle_increment
    angles = np.arange(ang,andf+inc,inc)

    vec = np.zeros((scan.shape[0],2))
    vec[:,0] = scan*np.cos(angles) + pos.linear.x
    vec[:,1] = scan*np.sin(angles) + pos.linear.y
    vec = vec[~np.isnan(vec).any(axis=1), :]
    vec = vec[np.isfinite(vec).any(axis=1), :]
    clusters = db.fit_predict(vec)
    
    for i in range(clusters.max() + 1):
        pdc = vec[clusters==i,:]
        cdc = np.mean(pdc, axis=0) 
        b = True
        if len(centers) == 0 and pdc.shape[0] < 10 and dis(cdc, [pos.linear.x,pos.linear.y]) < 1.5:
            b = True
        elif len(centers) == 0:
            b = False
        for j in centers:
            if dis(cdc,j) < 0.5 or pdc.shape[0] > 10 or dis(cdc, [pos.linear.x,pos.linear.y]) > 1.5:
                b = False
        if b:
            centers.append(cdc)
            ps = PointStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = 'turtlebot3_burger'
            ps.point.x = cdc[0]
            ps.point.y = cdc[1]
            pub.publish(ps)


    plt.clf()
    plt.scatter(*(vec.T),c = clusters)
    plt.scatter(pos.linear.x,pos.linear.y)
    if len(centers) > 0:
        plt.scatter(*(np.array(centers).T))
    plt.axis("equal")
    plt.draw()
    plt.pause(0.00000000001)

def main():
    global pos
    
    odom = rospy.Subscriber('/odom',Odometry,callbackOdom)
    scanS = rospy.Subscriber('/scan',LaserScan,callbackScan)

    plt.ion()
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass