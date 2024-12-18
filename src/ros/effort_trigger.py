import rospy
from geometry_msgs.msg import WrenchStamped

class NodoFuerza:
    def __init__(self, umbral_fuerza=5.0):
        rospy.init_node("nodo_vision", anonymous=True)
        self.force_threshold = umbral_fuerza
        self.force_detected = False
        rospy.Subscriber('/robot/wrench', WrenchStamped, self.__cb_wrench)
        rospy.sleep(.5)
        rospy.loginfo("Nodo de fuerza inicializado.")

    def __cb_wrench(self, msg: WrenchStamped) -> None:
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        force_magnitude = (force_x**2 + force_y**2 + force_z**2)**0.5
        
        # rospy.loginfo(force_magnitude)        
        # rospy.loginfo(f"Magnitud de la fuerza: {force_magnitude:.2f}")        
        # print(f"Magnitud de la fuerza: {force_magnitude:.2f}", end="\r")
        # print(force_magnitude)

        if force_magnitude > self.force_threshold:
            self.force_detected = True

    def golpe_detectado(self):
        aux: bool = False
        if self.force_detected:
            rospy.loginfo(f"Golpe detectado")
            aux = self.force_detected
            self.force_detected = False
            return aux
        
        return self.force_detected