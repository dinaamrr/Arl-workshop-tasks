
import rospy
from std_msgs.msg import String

def response_to_pub(data):

    name, age, height = data.data.split(",")
    print(name)
    print(age)
    print(height)

def personalInfo_pub():

    rospy.init_node('data_processing', anonymous=True)
    rospy.Subscriber("raw_data", String, response_to_pub)


    rospy.spin()



def talker(name, age, height):

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():

        pub.publish({name, age, height})
        rospy.loginfo({name, age, height})
        rate.sleep()



if __name__ == '__main__':
    pub = rospy.Publisher('user_info', String, queue_size=10)
    personalInfo_pub()
