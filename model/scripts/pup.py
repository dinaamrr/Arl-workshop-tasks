#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

velocity = input('Please Write the velocity ')
delta = input('Please Write steering_angle ')

def talk_to_me():
    rospy.init_node('model_input')
    pup = rospy.Publisher('/input_data', String, queue_size=10)
    rate = rospy.Rate(1)
    my_msg = "velocity: " + velocity + ", steering_angle: " + delta
    print(my_msg)
    while not rospy.is_shutdown():
        pup.publish(my_msg)
        rate.sleep()
    

if __name__ == "__main__":
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass