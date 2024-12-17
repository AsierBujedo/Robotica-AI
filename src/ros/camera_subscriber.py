import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from copy import deepcopy

bridge = CvBridge()

class NodoCamara:
    def __init__(self) -> None:
        self.cv_image = None
        rospy.Subscriber('/usb_cam/image_raw', Image, self.__cb_image)
        rospy.loginfo("Nodo de la cámara inicializado.")

    def __cb_image(self, image: Image):
        self.cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

    def read_image(self):
        if self.cv_image is not None:
            return True, deepcopy(self.cv_image)
        return False, None
