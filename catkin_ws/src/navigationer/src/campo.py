#!/usr/bin/env python3
# Campos potenciales





import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist



# Introducimos ahora dos nuevos tipos de mensajes relevantes. Imagen  y nube de puntos, Ambos mensajes propios de ros, 
# se muestran algunas librerías que hacen más fácil su utilización en Python
# https://www.ros.org/
# http://wiki.ros.org/ros_numpy
# http://wiki.ros.org/cv_bridge
# 


# En este notebook una simple maquina de estados para evasión de obstáculos


rospy.init_node('evasion_notebook_node') 
base_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Nos valdremos de nuestra vieja función de movimiento, asi como la clase Laser para obtener las lecturas del sensor (revisar notebook 1)

# Simultáneamente , es publicado en el tópico "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"
# , información de la nube de puntos, en forma de un mensaje tipo  PointCloud2

from sensor_msgs.msg import LaserScan


class Laser():
    u"""Class that handles laser information"""

    def __init__(self):
        # Register the _laser_cb method as a callback to the laser scan topic events
        self._laser_sub = rospy.Subscriber ('scan',
                                           LaserScan, self._laser_cb)
        self._scan_data = None

    def _laser_cb (self, msg):
        # Laser scan callback function
        self._scan_data = msg

    def get_data(self):
        u"""Function to get the laser value"""
        return self._scan_data

x = 0
y = 0
th = 0
import tf
def newOdom (msg):
    global x
    global y
    global th
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    th=euler[2]

xcl = 0.0
ycl = 0.0

#READ CLICK POINT
from geometry_msgs.msg import PointStamped
def readPoint(punto):
     global xcl , ycl
     
     xcl =punto.point.x
     ycl =punto.point.y


from nav_msgs.msg import Odometry

sub= rospy.Subscriber("/odom",Odometry,newOdom)



pub = rospy.Publisher('/cmd_vel_camp', Twist, queue_size=1)
sub = rospy.Subscriber('/objective',PointStamped,readPoint)


# # POTFIELDS


laser= Laser()
time.sleep(1)

def get_Rep_Force():
    data=laser.get_data()
    lec=np.asarray(data.ranges)
    lec[np.isinf(lec)]=13.5
    deltaang=   ( data.angle_max-data.angle_min  )   /len(data.ranges)
    laserdegs=  np.arange(data.angle_min,data.angle_max,deltaang)
    Fx=  0.0
    Fy = 0.0
    for i,deg in enumerate(laserdegs):
        if i >=360:break 

        if (lec[i] < 4.9): ###TUNABLE
            Fx = Fx + (1/lec[i])**2 * np.cos(deg)
            Fy = Fy + (1/lec[i])**2 * np.sin(deg)
    Fth= np.arctan2(Fy,(Fx))+np.pi
    #print('FxFyFth',Fx,Fy,Fth*180/np.pi)
    Fmag= np.linalg.norm((Fx,Fy))
    return Fx,Fy,Fmag,Fth
    


def get_Att_Force():
    xy,xycl=np.array((x,y)) ,   np.array((xcl,ycl))
    euclD=np.linalg.norm(xy-xycl)
    #print("xrob,yrob, throbot",x,y,th*180/3.1416)
    #print("xclick,yclick",xcl,ycl,"euclD",euclD)

    Fatrx =( -x + xcl)/euclD
    Fatry =( -y + ycl)/euclD      
    Fatrth=np.arctan2(Fatry, Fatrx) 
    Fatrth=Fatrth-th
    Fmagat= np.linalg.norm((Fatrx,Fatry))
    #print ('Fatx, Fatry, Fatrth',Fatrx,Fatry,(Fatrth)*180/np.pi )
    return  Fatrx, Fatry , Fmagat, Fatrth, euclD


def get_Speed(Ftotx,Ftoty,Ftotth):
    speed=Twist()
    if( abs(Ftotth) < .1) :
        
        speed.linear.x=0.13
        #print('lin')
        speed.angular.z=0

    else:
        
        if Ftotth < 0:                            
                if (abs( Ftotth ) < np.pi/2):
                    speed.linear.x= 0.025
                    speed.angular.z=-0.25
                    #print('open curve')
                
                else:                    
                    #print('Vang-')
                    speed.linear.x=  0.0
                    speed.angular.z=-0.5
                        
        if Ftotth > 0:
                if (abs( Ftotth ) < np.pi/2):
                    speed.linear.x= 0.05
                    speed.angular.z=.25
                    #print('open curve')
                
                else:
                    #print('Vang+')
                    speed.linear.x=  0.0
                    speed.angular.z= 0.5
    return speed


import random
rate = rospy.Rate(10)
counter=0
me_muero = False
charger = [0,0]

inicio = True
n_s = 500
while not rospy.is_shutdown():
        
        counter+=1
        xy,xycl=np.array((x,y)) ,   np.array((xcl,ycl))
        print(xycl)
        euclD=np.linalg.norm(xy-xycl)
        if counter % 100 == 0:
            print(counter)
        #print(counter)
        Fx,Fy,Fmag,Fth          =get_Rep_Force()
        Fatx,Faty,Fmagat,Fatrth, euclD =get_Att_Force()
        #######TUNABLE
    
        Ftotx= Fmag*np.cos(Fth)*.004   +    Fmagat*np.cos(Fatrth)# * signer
        Ftoty= Fmag*np.sin(Fth)*.004    +    Fmagat*np.sin(Fatrth)# * signer
        Ftotth=np.arctan2(Ftoty,Ftotx)
        ###ANGLE NORMALIZATION
        if ( Ftotth> np.pi ):     Ftotth=       -np.pi-    (Ftotth-np.pi)
        if (Ftotth < -np.pi): Ftotth= (Ftotth     +2 *np.pi)
        if counter==1000000:
            print('Ftotxy',Ftotx,Ftoty,Ftotth*180/np.pi, 'eulcD', euclD)
            counter=0
        ####
        speed=get_Speed(Ftotx,Ftoty,Ftotth)
        #if eculD < 2: speed.linear.x=0.2
        
        radioRobot = 0.1
        if euclD < radioRobot * 2: 
            #speed.linear.x = 0 
            #speed.angular.z = 0
            print("im here", euclD,(euclD < radioRobot))
            #me_muero = True
            if counter > n_s:
                inicio = False
                counter = 0
            #rospy.sleep(1)
            #break
        pub.publish(speed)
        #if me_muero:
         #   break
        
        rate.sleep()


# ENTREGAR


# # TAREA POR EQUIPOS


# # Programar al menos 3 conductas para el turltebot. 
# # UTILIZAR MAQUINAS DE ESTADOS  y SMACH
#  
#  ## 1.   De ellas será  una evasión de obstáculos modificada de la tarea 1 para esta vez "rodear" el obstáculo
#  (I.A. Clásica)
#  
#  ## 2.  Implemente navegación reactiva utilizando campos potenciales y SLAM
#  (Completamente Reactivo).
#  
#  ## 3.  Implemente un "árbtiro" que sea capaz de detectar que el agente se encuentra en un mínimo local y repcuperar la movilidad.(Híbrido)
#  
#  ## 4. Utililzando tf2_ros  Obtenga un estimado de las coordenadas más "seguras".
#  ### se sabe que las patas de las  mesas un son punto particularmente peligroso...
#  
#  Publique una transformada (tf ) en posiciones cuyas lecturas de lidar permiten suponer la presencia de una mesa o silla ... patas .
#  
#  
#  
#  
#  
#  
#  


# # Obtener la ubicación del agente en el mapa... MAPAS SLAM:... 


import tf2_ros
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
def get_coords ():
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
    return trans



t=get_coords()
type (t)


broadcaster = tf2_ros.StaticTransformBroadcaster()


t.child_frame_id='here'


broadcaster = tf2_ros.StaticTransformBroadcaster()



broadcaster.sendTransform(t)





