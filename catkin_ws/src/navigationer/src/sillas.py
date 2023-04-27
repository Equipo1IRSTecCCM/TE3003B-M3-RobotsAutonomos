#!/usr/bin/env python3

# # Tarea 32 - Rodear objetos
# ## Equipo 3


# Primero se importan las librerias necesarias.


import rospy
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from sklearn.cluster import DBSCAN


# Luego se declaran variables globales.


pos = Twist()
db = DBSCAN(eps=0.25, min_samples=2)
rospy.init_node('scan_detect_fine')
rate = rospy.Rate(2)
pub = rospy.Publisher('/dopt_points',PointStamped,queue_size=10)
centers = []


# Declaramos el callback de Odometría el cual se encarga de dar la posición del robot de acuerdo a sus cálculos de las velocidades hechas en cada motor.


def callbackOdom(msg):
    global pos
    pos.linear.x = msg.pose.pose.position.x
    pos.linear.y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_,_,pos.angular.z) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


# Esta función sirve para dar la diferencia de distancia entre dos puntos en el plano cartesiano.


def dis(src, goal):
    return np.sqrt((goal[0]-src[0])**2 + (goal[1]-src[1])**2)


# Este callback verifica con el LiDAR los obstáculos parecidos a tubos como patas de sillas o mesas. Si se detecta un tubo con ciertas características, el centroide posible se publica a ROS.


def callbackScan(msg):
    global pos
    global db
    global pub
    global centers

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
            ps.header.frame_id = 'map'
            ps.point.x = cdc[0]
            ps.point.y = cdc[1]
            pub.publish(ps)


# Ésta es la función principal donde se inicializan los nodos suscriptores


def main():
    global pos
    
    odom = rospy.Subscriber('/odom',Odometry,callbackOdom)
    scanS = rospy.Subscriber('/scan',LaserScan,callbackScan)
    
    rospy.spin()


# Finalmente, para ejecutar el código oficialmente (sin antes haber ya ejecutado las celdas anteriores y lanzado el mundo de turtlebot en Gazebo), se ejecuta la siguiente celda para iniciar con el proceso:


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


