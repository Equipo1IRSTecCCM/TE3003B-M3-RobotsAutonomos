#!/usr/bin/env python3

# # Tarea 32 - Rodear objetos
# ## Equipo 3


# Primero se importan las librerias necesarias.


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from smach import State, StateMachine
import smach_ros
from tf.transformations import euler_from_quaternion
from time import sleep


# Luego se declaran variables globales.


pos = Twist()
pub = rospy.Publisher('/cmd_vel_rod',Twist,queue_size=10)
rospy.init_node('rodearsm')
rate = rospy.Rate(10)
nfb = False
ilb = False
sl = 0


# Declaramos el callback de Odometría el cual se encarga de dar la posición del robot de acuerdo a sus cálculos de las velocidades hechas en cada motor.


def callbackOdom(msg):
    global pos
    pos.linear.x = msg.pose.pose.position.x
    pos.linear.y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_,_,pos.angular.z) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


# Esta función sirve para dar la diferencia de ángulo entre dos ángulos en el plano cartesiano.


def smallest_angle_diff(t,s):
    a = t - s
    a -= 2*np.pi if a > np.pi else -2*np.pi if a < -np.pi else 0
    return a


# Este callback verifica con el LiDAR si tiene obstaculos enfrente y la pared a su izquierda. De los puntos de la pared calcula con regresion el error de ángulo el cual seria la pendiente.


def callbackScan(msg):
    global nfb
    global ilb
    global sl

    #Import scan
    scan = np.array(msg.ranges)

    #Get angles array
    ang = msg.angle_min - np.pi
    andf = msg.angle_max - np.pi
    inc = msg.angle_increment
    angles = np.arange(ang,andf+inc,inc)

    #Get only front detected points
    anglesf = angles[np.r_[-45:0]]
    scan_f = scan[np.r_[-45:0]]
    anglesf = anglesf[~np.isnan(scan_f)]
    scan_f = scan_f[~np.isnan(scan_f)]
    anglesf = anglesf[np.isfinite(scan_f)]
    scan_f = scan_f[np.isfinite(scan_f)]

    #Get only left detected points
    anglesl = angles[np.r_[55:100]]
    scan_l = scan[np.r_[55:100]]
    anglesl = anglesl[~np.isnan(scan_l)]
    scan_l = scan_l[~np.isnan(scan_l)]
    anglesl = anglesl[np.isfinite(scan_l)]
    scan_l = scan_l[np.isfinite(scan_l)]

    #Get booleans if obstacle in front and wall in its left
    nfb = not scan_f[scan_f < 0.35].shape[0] > 0
    ilb = scan_l[scan_l < 0.35].shape[0] > 0

    #Calculate slope with regression and give it as error
    if scan_l.shape[0] >= 13:
        px = scan_l*np.cos(anglesl)
        py = scan_l*np.sin(anglesl)
        sl = np.sum((px - np.average(px))*(py -np.average(py)))/np.sum((px - np.average(px))**2)


# El estado de giro solo se activa cuando tiene obstaculo enfrente.


class Turn(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t'])
    def execute(self, ud):
        global pub
        global rate
        global nfb
        global ilb
        vel = Twist()
        vel.angular.z = -np.pi/2
        pub.publish(vel)
        rate.sleep()
        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"


# El estado de avance y giro se activa cuando no hay obstaculos enfrente ni a su izquierda.


class AdvanceTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t'])
    def execute(self, ud):
        global pub
        global rate
        global nfb
        global ilb
        vel = Twist()
        vel.angular.z = np.pi/2
        vel.linear.x = 0.11
        pub.publish(vel)
        rate.sleep()
        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"


# El estado de avance se activa cuando no hay obstáculo en frente y tiene pared a su izquierda. Para estar paralelo a la pared utiliza la pendiente como error donde 0 sería que está totalmente paralelo.


class Advance(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t'])
    def execute(self, ud):
        global pub
        global rate
        global nfb
        global ilb
        global sl
        vel = Twist()
        vel.angular.z = sl
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= 2.84 else 2.84*np.sign(vel.angular.z)
        vel.linear.x = 0.11
        pub.publish(vel)
        rate.sleep()
        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"


# Ésta es la función principal donde se inicializan los nodos suscriptores y la máquina de estados.


def main():
    global pos
    global rate
    
    odom = rospy.Subscriber('/odom',Odometry,callbackOdom)
    scanS = rospy.Subscriber('/scan',LaserScan,callbackScan)
    
    sm = StateMachine(outcomes=['success','failure'])
    sm.userdata.sm_input = 0

    with sm:

        StateMachine.add('TURN', Turn(), transitions={'t':'TURN','at':'ADVANCETURN','a':'ADVANCE'})
        StateMachine.add('ADVANCE', Advance(), transitions={'t':'TURN','at':'ADVANCETURN','a':'ADVANCE'})
        StateMachine.add('ADVANCETURN', AdvanceTurn(), transitions={'t':'TURN','at':'ADVANCETURN','a':'ADVANCE'})
    
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT/ROD')
    #sis.start()
    
    outcome = sm.execute()
    
    if outcome == 'success':
        sis.stop()
    else:
        sleep(1)


# Finalmente, para ejecutar el código oficialmente (sin antes haber ya ejecutado las celdas anteriores y lanzado el mundo de turtlebot en Gazebo), se ejecuta la siguiente celda para iniciar con el proceso:


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


