#!/usr/bin/env python

import rospy
from pinger_sonar.msg import PingerSonarDistanceSimple

from brping import Ping1D

class PingerSonarClass:

    def __init__(self):
        
        self.myPing = Ping1D()
        self.device = "/dev/ttyUSB0"
        self.baudrate = 9600
        self.myPing.connect_serial(self.device, self.baudrate)

        self.speed_m_p_s = 1500
        self.myPing.set_speed_of_sound(self.speed_m_p_s * 1000) # Speed of Sound in mm/s (1,500,000 mm/s for water)
        self.data = []
        
        self.pub = rospy.Publisher("pinger", PingerSonarDistanceSimple, queue_size = 10)
        self.receive_sonar_data()

    def receive_sonar_data(self):

        while not rospy.is_shutdown():

            self.data = self.myPing.get_distance_simple()
            if self.data:
                msg = PingerSonarDistanceSimple()
                msg.header.stamp = rospy.Time.now() 
                msg.header.frame_id = "pinger_sonar"
                msg.distance_m = self.data["distance"] / 1000 # mm to m
                msg.confidence_p = self.data["confidence"]
                self.pub.publish(msg)

if __name__ == '__main__':

    rospy.init_node("pinger_sonar_node", anonymous=True)
    PingerSonarClass()
    rospy.spin()