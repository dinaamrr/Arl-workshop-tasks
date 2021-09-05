
#!/usr/bin/python
import rospy
from exercise_one.msg import raw_data
import random

def personalData():
    #Creating a Node
    rospy.init_node("data_publisher_node",anonymous=True)
    #Creating a publisher topic
    pub=rospy.Publisher("data_publisher_topic", raw_data,queue_size=10)
    rate=rospy.Rate(1) #1Hz

    my_data=raw_data()

    my_data.name="Rose"
    my_data.age=20
    my_data.height=170

    while not rospy.is_shutdown():
       
        rospy.loginfo("Name: %s,age: %d ,height: %d", my_data.name,my_data.age,my_data.height)
        pub.publish(my_data)
        rate.sleep()


if __name__ =='__main__':
    try:
        personalData()
    except rospy.ROSInterruptException:
        pass
