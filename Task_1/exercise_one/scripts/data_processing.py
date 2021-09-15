
import rospy
from std_msgs.msg import String

def response_to_pub(data):
   name = data.data.split(",")
   print(name)
   age = data.data.split(",")
   print(age)
   height = data.data.split(",")
   print(height)



def personalInfo_pub():
    rospy.init_node('data_subscriber_node',anonymous=True)
    sub=rospy.Subscriber("raw_data",String,response_to_pub)
    rospy.spin()


if __name__ =="__main__" :
    personalInfo_pub()
