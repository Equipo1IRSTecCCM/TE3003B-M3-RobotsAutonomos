#!/usr/bin/env python3

# # Tarea 31 como Máquina de Estados
# ## Equipo 3


# Primero se importan las librerias necesarias.


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from smach import State, StateMachine
import smach_ros
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
from time import sleep

xcl = 0.0
ycl = 0.0
puntos = [[xcl,ycl]]

# Luego se declaran variables globales.
def obj_cb(msg):
    global xcl
    global ycl
    global puntos

     
    xcl =msg.point.x
    ycl =msg.point.y
    puntos = [[xcl,ycl]]

pos = Twist()
break_v = 0
turn_b = 0
pub = rospy.Publisher('/cmd_vel_eva',Twist,queue_size=10)
sub_obj = rospy.Subscriber('/objective',PointStamped, obj_cb)
rospy.init_node('fsm_ra')
rate = rospy.Rate(10)



# Esto es solo para verificar la lista de puntos si aún tiene o no.


puntos


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


# Este callback verifica con el LiDAR los obstaculos más cercanos los cuales si se detecta alguno calcula la cantidad de freno y desviación para evitar colisión.


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


# Este es el primer estado el cual envía al robot la velocidad de giro necesaria para alcanzar el ángulo mínimo entre el turtlebot y el punto objetivo, y así iniciar haciendo un recorrido en línea recta.


class Turn(State):
    def __init__(self):
        State.__init__(self, outcomes=['turnr','fowardr','success'])
    def execute(self, ud):
        global pos
        global pub
        global rate
        global puntos
        if len(puntos) == 0:
            return 'success'
        goal = puntos[0]
        print(goal)
        vel = Twist()
        ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
        vel.angular.z = smallest_angle_diff(ang,pos.angular.z)*2
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= 2.84 else 2.84*np.sign(vel.angular.z)
        pub.publish(vel)
        rate.sleep()
        if abs(smallest_angle_diff(ang,pos.angular.z)) > 0.01:
            return 'turnr'
        else:
            vel.angular.z = 0
            pub.publish(vel)
            rate.sleep()
            pub.publish(vel)
            rate.sleep()
            return 'fowardr'


# El segundo estado consiste en avanzar y corregir el rumbo si se está desviando a partir de un giro muy discreto. Si hay obstaculo aquí se aplica el freno y la desviación.


class Foward(State):
    def __init__(self):
        State.__init__(self, outcomes=['turnr','fowardr','success'])
    def execute(self, ud):
        global pos
        global pub
        global rate
        global puntos
        global break_v
        global turn_b
        goal = puntos[0]
        print(goal)
        vel = Twist()
        ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
        vel.angular.z = smallest_angle_diff(ang,pos.angular.z)*2 if turn_b == 0 else turn_b*(1-break_v) if break_v >= 0 else 0
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= 2.84 else 2.84*np.sign(vel.angular.z)
        dis = np.sqrt((pos.linear.x - goal[0])**2 + (pos.linear.y - goal[1])**2)
        vel.linear.x = dis*(break_v)
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.22 else 0.22*np.sign(vel.linear.x)*abs(break_v)
        pub.publish(vel)
        rate.sleep()
        if dis > 0.01:
            return 'fowardr'
        else:
            vel.angular.z = 0
            vel.linear.x = 0
            pub.publish(vel)
            rate.sleep()
            pub.publish(vel)
            rate.sleep()
            if len(puntos) == 0:
                return 'success'
            else:
                return 'turnr'


# Ésta es la función principal donde se inicializan los nodos suscriptores y la máquina de estados.


def main():
    global pos
    global rate
    
    odom = rospy.Subscriber('/odom',Odometry,callbackOdom)
    scanS = rospy.Subscriber('/scan',LaserScan,callbackScan)
    
    sm = StateMachine(outcomes=['succeeded'])
    sm.userdata.sm_input = 0

    with sm:

        StateMachine.add('TURN', Turn(), transitions={'turnr':'TURN','fowardr':'FOWARD','success':'FOWARD'})
        StateMachine.add('FOWARD', Foward(), transitions={'turnr':'TURN','fowardr':'FOWARD','success':'TURN'})
    
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()
    
    outcome = sm.execute()
    '''
    if outcome == 'success':
        sis.stop()
    else:
        sleep(1)
    '''


# Finalmente, para ejecutar el código oficialmente (sin antes haber ya ejecutado las celdas anteriores y lanzado el mundo de turtlebot en Gazebo), se ejecuta la siguiente celda para iniciar con el proceso:


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


