import rospy
from geometry_msgs.msg import Wrench

class NodoFuerza:
    def __init__(self, umbral_fuerza=5.0):
        self.force_threshold = umbral_fuerza
        self.force_detected = False
        rospy.Subscriber('/robot/wrench', Wrench, self.__cb_wrench)
        rospy.loginfo("Nodo de fuerza inicializado.")

    def __cb_wrench(self, msg: Wrench):
        force_x = msg.force.x
        force_y = msg.force.y
        force_z = msg.force.z
        force_magnitude = (force_x**2 + force_y**2 + force_z**2)**0.5

        if force_magnitude > self.force_threshold:
            rospy.loginfo(f"Golpe detectado: {force_magnitude:.2f} N")
            self.force_detected = True
        else:
            self.force_detected = False

    def golpe_detectado(self):
        return self.force_detected
