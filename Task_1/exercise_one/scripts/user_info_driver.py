
import rospy
from std_msgs.msg import String
def personalData():
    pub = rospy.Publisher('raw_data', String, queue_size=10)
    rospy.init_node('data_publisher_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        pers_str = "name:Rose, age:20, height:170"
        rospy.loginfo(pers_str)
        pub.publish(pers_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        personalData()
    except rospy.ROSInterruptException:
        pass
