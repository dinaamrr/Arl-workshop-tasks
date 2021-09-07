import rospy
from std_msgs.msg import String

def response_to_pub(data):

    name = data.data.split(",")
    age = data.data.split(",")
    height = data.data.split(",")
    talker(name, age, height)


def personalInfo_pub():

    rospy.init_node('data_processing', anonymous=True)

    rospy.Subscriber("raw_data", String, callback)

    rospy.spin()




def talker(name, age, height):

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        name_pub.publish(name)
        rospy.loginfo(name)

        age_pub.publish(age)
        rospy.loginfo(age)

        height_pub.publish(height)
        rospy.loginfo(height)
        rate.sleep()

