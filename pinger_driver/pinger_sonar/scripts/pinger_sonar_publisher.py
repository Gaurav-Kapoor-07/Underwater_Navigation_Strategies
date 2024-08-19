#!/usr/bin/env python

import rospy
from pinger_sonar.msg import PingerSonar

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
        
        self.pub = rospy.Publisher("pinger", PingerSonar, queue_size = 10)
        self.receive_sonar_data()

    def receive_sonar_data(self):

        while not rospy.is_shutdown():

            self.data = self.myPing.get_profile()
            if self.data:
                msg = PingerSonar()
                msg.header.stamp = rospy.Time.now() 
                msg.header.frame_id = "pinger_sonar"
                msg.distance_m = self.data["distance"] / 1000 # mm to m
                msg.confidence_p = self.data["confidence"]
                msg.transmit_duration_s = self.data["transmit_duration"] / 1000000 # us to s
                msg.ping_number = self.data["ping_number"]
                msg.scan_start_m = self.data["scan_start"] / 1000 # mm to m
                msg.scan_length_m = self.data["scan_length"] / 1000 # mm to m
                msg.gain_setting = self.data["gain_setting"]
                msg.profile_data_length = len(self.data['profile_data'])
                msg.profile_data = self.data['profile_data']
                self.pub.publish(msg)

if __name__ == '__main__':

    rospy.init_node("pinger_sonar_node", anonymous=True)
    PingerSonarClass()
    rospy.spin()