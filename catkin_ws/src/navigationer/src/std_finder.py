#!/usr/bin/env python3

#Find std_of last position
import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf
from variables import *
class std_finder:
    def __init__(self,n_iter) -> None:
        self.arr = np.zeros((n_iter,2))
        self.n_iter = n_iter
        self.go = False
        self.std_dev = np.inf
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.count = 0
        self.pub_debug = rospy.Publisher("/nav/debugger2",Float32,queue_size=10)
        self.pub_go = rospy.Publisher('nav/go',Bool,queue_size=10)
        self.pub_std = rospy.Publisher('/nav/std',Float32,queue_size=10)
    def start(self):
        self.sub_odom = rospy.Subscriber("/odom",Odometry,self.newOdom)
        self.sub_restart = rospy.Subscriber('/nav/restart',Bool, self.restart_callback)

    def newOdom (self,msg):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.th=euler[2]
    def run(self,rate_hz):
        rate  = rospy.Rate(rate_hz)
        self.count = 0
        while not rospy.is_shutdown():
            self.go = self.count > self.n_iter
            if self.arr.shape[0] >= self.n_iter:
                #Sacar std, guardar nuevos datos
                self.arr[:self.n_iter-1] = self.arr[1:self.n_iter]
                self.arr[-1] = np.array([self.x,self.y])
                self.std_dev = (np.std(self.arr[:,0]) + np.std(self.arr[:,1]))/2
            else:
                #añadir a queue
                self.arr.append(np.array([self.x,self.y]))
            self.pub_std.publish(self.std_dev)
            self.pub_go.publish(self.go)
            rate.sleep()
            self.count += 1
        #rospy.spin()
    def restart_callback(self,msg):
        self.go = False
        self.count = 0
def main():
    rospy.init_node('std_finder') 
    #rate = rospy.Rate(rate_hz)
    std_f = std_finder(n_iter)
    std_f.start()
    std_f.run(rate_hz)
if __name__ == '__main__':
    main()
