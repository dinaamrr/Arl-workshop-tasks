#! /usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import String, Int64
from model.msg import model



def cb(msg):
    messagee = msg.data
    messagee = messagee.replace("velocity: ","")
    messagee = messagee.replace(" steering_angle: ","")
    ls = messagee.split(',')
    print("Subscriber data: ")
    rospy.loginfo(msg)

    #Kinematic_Bicycle_Model
    class Bicycle():
        def __init__(self):
            self.xc = 0
            self.yc = 0
            self.theta = 0
            self.beta = 0
            self.xc_dot = 0
            self.yc_dot = 0
            self.theta_dot = 0
            self.L = 2
            self.lr = 1.2
            self.sample_time = 0.01

    class Bicyclee(Bicycle):
        def step(self, v, delta):
            
            #bicycle Kinematics
            self.xc_dot = v* np.cos(self.theta + delta)
            self.yc_dot = v* np.sin(self.theta + delta)
            self.theta_dot = v* np.cos(self.theta)* np.tan(delta)/self.L

            #update the state variables
            self.xc += self.xc_dot * self.sample_time
            self.yc += self.yc_dot * self.sample_time
            self.theta += self.theta_dot * self.sample_time
            self.beta = np.arctan(self.lr*np.tan(delta)/self.L)

    modell = Bicyclee()
    modell.step(float(ls[0]), float(ls[1]))
    
    print("Publisher data: ")

    my_msg = model()
    my_msg.velocity   = float(ls[0])
    my_msg.delta      = float(ls[1])
    my_msg.xc_dot     = modell.xc_dot
    my_msg.ycdot      = modell.yc_dot  
    my_msg.theta_dot  = modell.theta_dot
    my_msg.beta       = modell.beta
    my_msg.theta      = modell.theta
    my_msg.xc         = modell.xc
    my_msg.yc         = modell.yc
    pup.publish(my_msg)

    

def sub_pup():
    rospy.init_node('data_processing')
    global pup
    pup = rospy.Publisher('/output_data', model, queue_size=100)
    rospy.Subscriber('/input_data', String, callback=cb)
    rospy.spin()

if __name__ == "__main__":
    try:
        print("Running...")
        sub_pup()
    except rospy.ROSInterruptException:
        pass
