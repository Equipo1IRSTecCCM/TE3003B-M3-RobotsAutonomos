#!/usr/bin/env python3

# Find objective ppoints
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import tf
import cv2
from queue import Queue
class map:
    def __init__(self) -> None:
        #Map
        self.occupancy_grid = None
        self.info = None
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        #Odom
        self.x = None
        self.y = None
        self.th = None
        self.sub_odom = rospy.Subscriber('/odom',Odometry, self.callback_odom)
        self.position = None

        #Nav
        self.nearest = None
        self.dist = -1
        self.change = True
        self.already_visit = []
        self.frontiers = np.array([])
        self.pub_point = rospy.Publisher('/objective', PointStamped, queue_size=10)

        #Debug
        self.pub_debug = rospy.Publisher('/debug_map',Float32, queue_size=10)
    def callback_map(self, msg):
        self.info = msg.info
        # Convert the data to a numpy array
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        if self.x != None:
            #print(self.x,self.y)
            x_map = int((self.x-self.info.origin.position.x) / self.info.resolution)
            y_map = int((self.y-self.info.origin.position.y) / self.info.resolution)
            self.position = (y_map,x_map)
            self.occupancy_grid[y_map-2:y_map+2,x_map-2:x_map+2] = 50
            if self.change:
                self.find_nearest()
                self.occupancy_grid[self.nearest[0]-2:self.nearest[0]+2,self.nearest[1]-2:self.nearest[1]+2] = 75
    def callback_odom(self,msg):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.th=euler[2]
        try:
            x_obj = self.nearest[1]*self.info.resolution + self.info.origin.position.x
            y_obj = self.nearest[0]*self.info.resolution + self.info.origin.position.y
            dist = np.sqrt((self.x - x_obj)**2 + (self.y - y_obj)**2)
            if dist < 0.5:
                self.change = True
            else:
                self.change = False
        except:
            pass
    def show(self):
        try:
            heatmap = plt.imshow(self.occupancy_grid, cmap='hot', interpolation='nearest')
            plt.title("Time: {}".format(rospy.get_time()))
            plt.show(block=False)
            plt.pause(1.9)
            plt.close()
        except:
            pass
    def find_nearest(self, display = False):
        
        img =np.reshape(self.occupancy_grid,(self.occupancy_grid.shape[0],self.occupancy_grid.shape[1],1))
        
        img[img == 100] = 255
        img[img == 0] = 100
        img[img == -1] = 0
        img = img.astype(np.uint8)
        front = cv2.Canny(img,50,150)
        if display:
            heatmap = plt.imshow(front, cmap='hot', interpolation='nearest')
            plt.title("Time: {}".format(rospy.get_time()))
            plt.show(block=False)
            plt.pause(1.9)
            plt.close()
        indices = np.array(np.where(front == 255))
        distances = np.sqrt((indices[0] - self.position[0])**2 + (indices[1] - self.position[1])**2)

        indices = indices[:,distances > 10]
        distances = distances[distances > 10]
        
        score = 24
        min_score = 25
        min_idx = None
        count = 0
        while count < distances.shape[0]/10 or score < 2:
            idx = distances.argmax()
            self.nearest = indices[:,idx]
            score = self.in_danger_zone(self.nearest)
            if score < min_score:
                min_score = score
                min_idx = self.nearest
            distances[idx] = -1
            count+=1

        if score > min_score:
            score = min_score
            self.nearest = min_idx
        
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.point.x = self.nearest[1]*self.info.resolution + self.info.origin.position.x
        point.point.y = self.nearest[0]*self.info.resolution + self.info.origin.position.y
        
        self.pub_point.publish(point)
        
    def in_danger_zone(self,idx,window_radius = 3):
        window = self.occupancy_grid[idx[0]-window_radius:idx[0]+window_radius,idx[1]-window_radius:idx[1]+window_radius]
        score = np.array(np.where(window == 100)).shape[1]
        
        return score

    def not_visited(self):
        meter = int(1 / self.info.resolution/2)
        for p in self.already_visit:
            #Check every visited point
            #if its within a meter of one of them, send false
            pass
        

if __name__ == '__main__':
    rospy.init_node("MapReader")
    m = map()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        #print(m.occupancy_grid)
        #m.show()
        
        pass