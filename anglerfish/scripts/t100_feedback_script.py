#!/usr/bin/python

#Anglerfish has 6 T100 Thrusters with Blue ESCs.  Each thruster had its firmware manually updated 
#(using Arduino and AVRdude in Ubuntu) to acquire a unique I2C address.
#See BlueRobotics T100 instructions on how to program ESCs.
#I2C address' are: 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E
#since launch file doesn't support hex: 41, 42, 43, 44, 45, 46

import sys
import smbus
import time
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Header
from std_msgs.msg import String
from anglerfish.msg import t100_thruster_feedback

bus = smbus.SMBus(1)

#Temperature calculator
def temperature_calculate(temp_reg):

	# THERMISTOR SPECIFICATIONS
	# resistance at 25 degrees C
	THERMISTORNOMINAL = 10000      
	# temp. for nominal resistance (almost always 25 C)
	TEMPERATURENOMINAL =  25   
	# The beta coefficient of the thermistor (usually 3000-4000)
	BCOEFFICIENT = 3900
	# the value of the 'other' resistor
	SERIESRESISTOR = 3300 

	resistance = SERIESRESISTOR/(65535/float(temp_reg)-1)
	steinhart = resistance / THERMISTORNOMINAL  # (R/Ro)
	steinhart = math.log(steinhart)                 # ln(R/Ro)
	steinhart /= BCOEFFICIENT                  # 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15) # + (1/To)
	steinhart = 1.0 / steinhart                 # Invert
	steinhart -= 273.15    

	return steinhart

#voltage calculation
def voltage(voltage_raw):
	return (voltage_raw*.0004921)

#current calculation taken from BlueRobotics
def current(current_raw):
	return ((current_raw-32767)*.001122)

#take pulse count over time, calculate RPM
def RPM(pulse, _rpmTimer):

	_rpm = (float(pulse)/((time.clock()-_rpmTimer)*120))*60

	_rpmTimer = time.clock()
	return _rpm, _rpmTimer

class read_registers():

	def __init__(self):
		#ROS params for definging node for each thruster
		T100_ADDR = rospy.get_param('~register')
		T100_name = rospy.get_param('~name')

		#T100 registers used for RPM, voltage, temp, current (2 bytes each)
		T100_PULSE_COUNT_1   = 0x02
		T100_PULSE_COUNT_2   = 0x03
		T100_VOLTAGE_1       = 0x04
		T100_VOLTAGE_2       = 0x05
		T100_TEMPERATURE_1   = 0x06
		T100_TEMPERATURE_2   = 0x07
		T100_CURRENT_1       = 0x08
		T100_CURRENT_2       = 0x09
		#T100 register for detecting if T100 is alive (bool)
		T100_ALIVE           = 0x0A

		self.ROV_pub = rospy.Publisher(T100_name, t100_thruster_feedback, queue_size=1)

		t100 = t100_thruster_feedback()

		rate = rospy.Rate(15)	

		_rpmTimer = 0.0

		while not rospy.is_shutdown():
			
			#Check to see if thruster is alive
			if bus.read_byte_data(T100_ADDR, T100_ALIVE):

				#read pulse reg to calculate RPM
				pulse1_read = bus.read_byte_data(T100_ADDR, T100_PULSE_COUNT_1)
				pulse2_read = bus.read_byte_data(T100_ADDR, T100_PULSE_COUNT_2)
				
				#bit shift 2 bytes
			        pulse_count_reg = pulse1_read << 8 | pulse2_read
				
				#return RPM
				actual_RPM,_rpmTimer = RPM(pulse_count_reg, _rpmTimer)
				
				#read temp registers
				temp1_read = bus.read_byte_data(T100_ADDR, T100_TEMPERATURE_1)
				temp2_read = bus.read_byte_data(T100_ADDR, T100_TEMPERATURE_2)
	
			        thruster_temp_reg = temp1_read << 8 | temp2_read
				
				#return actual temp
				thruster_temp = temperature_calculate(thruster_temp_reg)

				#read voltage reg
				volt_1 = bus.read_byte_data(T100_ADDR, T100_VOLTAGE_1)
				volt_2 = bus.read_byte_data(T100_ADDR, T100_VOLTAGE_2)

			        voltage_temp_reg = volt_1 << 8 | volt_2

				#return actual voltage
				actual_voltage = voltage(voltage_temp_reg)
				
				#read current registers, bit shift
				curr_1 = bus.read_byte_data(T100_ADDR, T100_CURRENT_1)
				curr_2 = bus.read_byte_data(T100_ADDR, T100_CURRENT_2)

			        current_temp_reg = curr_1 << 8 | curr_2

				#return actual current
				actual_current = current(current_temp_reg)

				t100.t100_header = Header(
					stamp = rospy.get_rostime(),
					frame_id = str(T100_ADDR)
				)
				
				t100.temperature = thruster_temp
				t100.voltage = actual_voltage
				t100.current = actual_current 
				t100.rpm = actual_RPM
				t100.alive = "CONNECTED"

			else:
				t100.alive = "DISCONNECTED"			

			self.ROV_pub.publish(t100)
			rate.sleep()

def main(args):
	rospy.init_node('t100_feedback_sensors', anonymous=True)

	read_registers()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)
