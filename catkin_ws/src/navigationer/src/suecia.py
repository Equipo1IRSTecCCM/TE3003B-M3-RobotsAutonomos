#!/usr/bin/env python3
# %%
import math
import numpy as np
import rospy
import smach
import smach_ros
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from variables import *

# %%
rospy.init_node('CesarArturoRamos') 
base_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
c2e = []
e2c = []
n_checks = 5
x = 0.0
y = 0.0
th = 0.0
obj_x = 0.0
obj_y = 0.0

# %%
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
sub_odom = rospy.Subscriber("/odom",Odometry,newOdom)

# %%


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
laser = Laser()  #instanciamos una clase 

# %%


# %%
def update_c2e(data):
    global c2e
    if len(c2e) >= n_checks:
        c2e[:n_checks-1,:] = c2e[1:n_checks,:]
        c2e[-1] = data
    else:
        c2e.append(data)
        if len(c2e) == n_checks:
            c2e = np.array(c2e)
def get_c2e():
    global c2e
    if len(c2e) >= n_checks:
        return (np.std(c2e[:,0]) + np.std(c2e[:,1]))/2
    else:
        return np.inf
def get_e2c():
    global e2c
    if len(e2c) >= n_checks:
        return (np.std(e2c[:,0]) + np.std(e2c[:,1]))/2
    else:
        return np.inf
def update_e2c(data):
    global e2c
    if len(e2c) >= n_checks:
        e2c[:n_checks-1,:] = e2c[1:n_checks,:]
        e2c[-1] = data
    else:
        e2c.append(data)
        if len(e2c) == n_checks:
            e2c = np.array(e2c)

# %%
go = False
def get_go(msg):
    global go
    go = msg.data

# %%
std_dev = 0.0
def get_std(msg):
    global std_dev
    std_dev = msg.data
    

# %%
class TwistSub:
    def __init__(self):
        self.data = Twist()
    def callback(self, msg):
        self.data = msg

# %%

def get_obj(msg):
    global obj_x
    global obj_y

    obj_x = msg.point.x
    obj_y = msg.point.y

# %%
x = 0
y = 0
th = 0


thr = 0.1
sub_go = rospy.Subscriber('nav/go',Bool,get_go)
sub_obj = rospy.Subscriber('/objective',PointStamped,get_obj)
sub_std = rospy.Subscriber('/nav/std',Float32,get_std)
pub_restart = rospy.Publisher('/nav/restart',Bool,queue_size=10)


# %%
class Campos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['estancado','rod','todoOk'])
        self.cmd_camp = TwistSub()
        self.sub_camp= rospy.Subscriber("/cmd_vel_camp",Twist,self.cmd_camp.callback)
    def execute(self, userdata):
        global go
        global std_dev
        global base_vel_pub
        global x
        global y
        global c2e
        global e2c
        print('Executing state Campos')
        base_vel_pub.publish(self.cmd_camp.data)
        if go:
            if std_dev < thr:
                pub_restart.publish(True)
                update_c2e(np.array([x,y]))
                std_c2e = get_c2e()
                std_e2c = get_e2c()
                if std_c2e < thr and std_e2c < thr:
                    return 'rod'
                return 'estancado'
        return 'todoOk'




class Evasion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['liberado','rod','todoOk'])
        self.cmd_eva = TwistSub()
        self.sub_eva = rospy.Subscriber("/cmd_vel_eva",Twist,self.cmd_eva.callback)

    def execute(self, userdata):
        print('Executing state Evasion')
        global go
        global std_dev
        global base_vel_pub
        global x
        global y
        global c2e
        global e2c
        base_vel_pub.publish(self.cmd_eva.data)
        if go:
            if std_dev > thr:
                pub_restart.publish(True)
                update_e2c(np.array([x,y]))
                std_c2e = get_c2e()
                std_e2c = get_e2c()
                if std_c2e < thr and std_e2c < thr:
                    return 'rod'
                return 'liberado'
        return 'todoOk'
class Rodear(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['liberado','todoOk'])
        self.cmd_rod = TwistSub()
        self.sub_rod = rospy.Subscriber("/cmd_vel_rod",Twist,self.cmd_rod.callback)

    def execute(self, userdata):
        print('Executing state Rodear')
        global go
        global std_dev
        global base_vel_pub
        global x
        global y
        global th
        global obj_x
        global obj_y
        base_vel_pub.publish(self.cmd_rod.data)
        data = laser.get_data()
        phi = np.arctan2(x - obj_x, y - obj_y)
        psi = th - phi

        scan = np.array(data.ranges)
        ang = data.angle_min
        andf = data.angle_max
        inc = data.angle_increment
        window = 60
        angles = np.arange(ang,andf+inc,inc)
        angles = angles[np.r_[psi - window:psi + window]]
        scan_f = scan[np.r_[psi - window:psi + window]]
        angles = angles[~np.isnan(scan_f)]
        scan_f = scan_f[~np.isnan(scan_f)]
        angles = angles[np.isfinite(scan_f)]
        scan_f = scan_f[np.isfinite(scan_f)]
        if scan_f[scan_f < 0.35].shape[0] > 0:
            return 'todoOk'
        else:
            return 'liberado'
        

#Crear clase de rodear
#Guardar punto donde se cambia de estado
#Si estos cambian poco, se pasa a este estado y se quita cuando tiene campo de visión sin estorbos con el objetivo

def main():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Camp', Campos(), 
                               transitions={'estancado':'Evasion', 
                                            'todoOk':'Camp',
                                            'rod':'Rodear'})
        smach.StateMachine.add('Evasion', Evasion(), 
                               transitions={'liberado':'Camp', 
                                            'todoOk':'Evasion',
                                            'rod':'Rodear'})
        smach.StateMachine.add('Rodear', Rodear(), 
                               transitions={'liberado':'Camp', 
                                            'todoOk':'Rodear'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT/ARB')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    sis.stop()

if __name__ == '__main__':
    main()


