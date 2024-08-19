#!/usr/bin/env python

import rospy
from sst_sensor_ring.msg import SSTSensorRing
import serial
import datetime

class SSTSensorRingClass:

    def __init__(self):

        self.ser = serial.Serial('/dev/ttyACM0')
        self.nmea_data = rospy.Publisher("sst_sensor_ring", SSTSensorRing, queue_size = 10)
        now = rospy.get_rostime()
        now_secs = now.secs + now.nsecs * 1e-9
        dt = datetime.datetime.fromtimestamp(now_secs).strftime('%d-%m-%Y_%H-%M-%S')
        filename = "sst_sensor_ring_" + str(dt) + ".csv"
        self.f = open(filename, "a")
        self.i = 0
        self.read_sensor_ring()
    
    def __del__(self):

        self.f.close()

    def read_sensor_ring(self):

        while not rospy.is_shutdown():

            data = self.ser.readline()
            
            self.f.write(data)

            data_split = data.split(",")

            msg = SSTSensorRing() 
            msg.header.frame_id = "sst_sensor_ring"
            msg.header.stamp = rospy.get_rostime()
            msg.string_header = data_split[0]
            msg.pressure_dBar = float(data_split[1])
            msg.temperature_degC = float(data_split[2])
            msg.conductivity_mS_per_cm = float(data_split[3])
            msg.oxygen_mBar = float(data_split[4])
            msg.sp440nm_percent = float(data_split[5])
            msg.sp460nm_percent = float(data_split[6])
            msg.sp485nm_percent = float(data_split[7])
            msg.sp500nm_percent = float(data_split[8])
            msg.sp550nm_percent = float(data_split[9])
            msg.sp640nm_percent = float(data_split[10])
            msg.sp660nm_percent = float(data_split[11])

            self.nmea_data.publish(msg)
            
            # if(data[5] == 'T'):
            #     self.i = 0
            #     self.i += 1
            #     timestamp = rospy.get_rostime()

            # elif(data[5] == 'A'):
            #     self.i += 1
            #     pressure_bits = int(data[7:12])
            #     pressure_bar = (pressure_bits - 16384) * (10.0 - 0) / 32768 + 0

            # elif(data[5] == 'C'):
            #     self.i += 1
            #     pressure_calculated_dbar = float(data[7:11])

            # elif(data[5] == 'D'):
            #     self.i += 1
            #     D_value = float(data[7:11])

            # if(self.i == 4):
            #     msg = SSTSensorRing() 
            #     msg.header.frame_id = "sst_sensor_ring"
            #     msg.header.stamp = timestamp
            #     msg.pressure_raw_mbar = pressure_bar * 1000
            #     msg.pressure_calculated_mbar = pressure_calculated_dbar * 100
            #     msg.D_value = D_value
            #     self.nmea_data.publish(msg)

                    
if __name__ == '__main__':

    rospy.init_node("sst_sensor_ring_node", anonymous=True)
    SSTSensorRingClass()
    rospy.spin()