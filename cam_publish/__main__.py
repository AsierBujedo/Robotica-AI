import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher:

    def __init__(self):
        rospy.init_node('camera_publisher')
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
        self.cap = cv2.VideoCapture(0)

    def publish_image(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture image")
                continue

            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.pub.publish(ros_image)
            rate.sleep()

if __name__ == "__main__":
    try:
        camera_pub = CameraPublisher()
        camera_pub.publish_image()
    except rospy.ROSInterruptException:
        pass
