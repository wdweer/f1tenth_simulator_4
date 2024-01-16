import rospy
from tf2_msgs.msg import TFMessage
rospy.init_node('tf_subscriber')

def callback(data):
    print(data)
tf_sub=rospy.Subscriber('/tf',TFMessage, callback)

rospy.spin()