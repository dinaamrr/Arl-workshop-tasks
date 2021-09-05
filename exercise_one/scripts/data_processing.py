
import rospy
from exercise_one.msg import raw_data

def response_to_pub(data):
    rospy.loginfo("Detail Info:\nName: %s\nage: %d\nheight: %d",data.name,data.age,data.height)



def personalInfo_pub():
    rospy.init_node("data_subscriber_node",anonymous=True)
    sub=rospy.Subscriber("data_publisher_topic",raw_data,response_to_pub)
    rospy.spin()


if __name__ =="__main__" :
    personalInfo_pub()
