#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

import ms5837

class ms5837PressureSensorClass:

    def __init__(self):
    
        self.sensor = ms5837.MS5837_02BA() 
        self.sensor.init()
        
        if not self.sensor.init():
            print("Sensor could not be initialized")
            exit(1)

        self.pressure_mbar_pub = rospy.Publisher("ms5803/pressure", Float64, queue_size = 10)
        self.depth_m_pub = rospy.Publisher("ms5803/depth", Float64, queue_size = 10)
        self.temperature_deg_C_pub = rospy.Publisher("ms5803/temperature", Float64, queue_size = 10)
        
        self.receive_pressure_data()

    def receive_pressure_data(self):

        while not rospy.is_shutdown():

            if self.sensor.read():

                self.pressure_mbar_pub.publish(self.sensor.pressure())
                self.depth_m_pub.publish(self.sensor.depth())
                self.temperature_deg_C_pub.publish(self.sensor.temperature()) 

if __name__ == '__main__':

    rospy.init_node("ms5837_pressure_sensor_node", anonymous=True)
    ms5837PressureSensorClass()
    rospy.spin()
