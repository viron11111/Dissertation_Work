#!/usr/bin/python

#MS5837 pressure and temperature sensor, located on back on Anglerfish
#translated from BlueRobotics Github: https://github.com/bluerobotics/BlueRobotics_MS5837_Library

#import roslib
import sys
import smbus
import time
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Header
from anglerfish.msg import ms5837

bus = smbus.SMBus(1)

MS5837_ADDR              = 0x76  #MS5837 address
MS5837_RESET             = 0x1E  
MS5837_ADC_READ          = 0x00	 #begin temp/pressure read
MS5837_PROM_READ         = 0xA0  #calibration values
MS5837_CONVERT_D1_8192   = 0x4A	 #first half of high resolution read
MS5837_CONVERT_D2_8192	 = 0x5A	 #2nd half of high resolution read

C = [] #used for calibration values
D = [] #holder for calibration values

def initialize_sensor():
        #reset device prior to initialization
        bus.write_byte(MS5837_ADDR, MS5837_RESET)
        time.sleep(.01)

        #grab factory set calibration vaules
        for i in range(0,7):
                D.append(bus.read_i2c_block_data(MS5837_ADDR,MS5837_PROM_READ + i*2,2)) #2 bytes read from MS5837
                C.append((D[i][0]<<8)| D[i][1]) #combine 2 bytes

def read():

        # test values from BlueRobotics
        #test = [0,34982,36352,20328,22354,26646,26146,0]

        #begin first half of conversion
        bus.write_byte(MS5837_ADDR, MS5837_CONVERT_D1_8192)
        time.sleep(.02)

        holder = bus.read_i2c_block_data(MS5837_ADDR, MS5837_ADC_READ,3)
        D1 = holder[0] << 16 | holder[1] << 8 | holder[2]

        #begin second half of conversion
        bus.write_byte(MS5837_ADDR, MS5837_CONVERT_D2_8192)
        time.sleep(.02)

        holder = bus.read_i2c_block_data(MS5837_ADDR, MS5837_ADC_READ,3)
        D2 = holder[0] << 16 | holder[1] << 8 | holder[2]

        return  calculate(D1,D2)


        SENS = C[1]*32768+(C[3]*dT)/256
        OFF = C[2]*65536+(C[4]*dT)/128

        #Temp and P conversion
        TEMP = 2000+dT*C[6]/8388608

        P = (D1*SENS/(2097152)-OFF)/(8192)

        #Second order compensation
        if((TEMP/100)<20):         #Low temp
                Ti = (3*(dT)*(dT))/(8589934592)
                OFFi = (3*(TEMP-2000)*(TEMP-2000))/2
                SENSi = (5*(TEMP-2000)*(TEMP-2000))/8
                if((TEMP/100)<-15):    #Very low temp
                        OFFi = OFFi+7*(TEMP+1500)*(TEMP+1500)
                        SENSi = SENSi+4*(TEMP+1500)*(TEMP+1500)
        elif((TEMP/100)>=20):    #High temp
                Ti = 2*(dT*dT)/(137438953472)
                OFFi = (1*(TEMP-2000)*(TEMP-2000))/16
                SENSi = 0




def calculate(D1,D2):

        dT = D2-C[5]*256 #difference between actual and reference temperature

        SENS = C[1]*32768+(C[3]*dT)/256
        OFF = C[2]*65536+(C[4]*dT)/128

        #Temp and P conversion
        TEMP = 2000+dT*C[6]/8388608

        P = (D1*SENS/(2097152)-OFF)/(8192)

        #Second order compensation
        if((TEMP/100)<20):         #Low temp
                Ti = (3*(dT)*(dT))/(8589934592)
                OFFi = (3*(TEMP-2000)*(TEMP-2000))/2
                SENSi = (5*(TEMP-2000)*(TEMP-2000))/8
                if((TEMP/100)<-15):    #Very low temp
                        OFFi = OFFi+7*(TEMP+1500)*(TEMP+1500)
                        SENSi = SENSi+4*(TEMP+1500)*(TEMP+1500)
        elif((TEMP/100)>=20):    #High temp
                Ti = 2*(dT*dT)/(137438953472)
                OFFi = (1*(TEMP-2000)*(TEMP-2000))/16
                SENSi = 0

        OFF2 = OFF-OFFi           #Calculate pressure and temp second order
        SENS2 = SENS-SENSi

        TEMP = (TEMP-Ti) #incorporate offset
        P = (((D1*SENS2)/2097152-OFF2)/8192) #pressure in mbar

        temperature = TEMP/100.0
        pressure = P/10.0
        fluid_density = 1029

        depth = ((pressure-1013.25)/(fluid_density*9.80665))*100  #depth in meters, positive equals depth in water, negative equals above water level
        altitude = (1-pow((pressure/1013.25),.190284))*145366.45  #another calculator for determining altitude, just because

        return depth, temperature, pressure

class measure_depth:

	def __init__(self):
		self.ROV_pub = rospy.Publisher('anglerfish', ms5837, queue_size=1)

		rov = ms5837()

		rate = rospy.Rate(20)	
		initialize_sensor()
		while not rospy.is_shutdown():
			depth_af,temperature_af, pressure_af = read()
		
			#rospy.loginfo(temperature_af)
			rov.header = Header(
				stamp = rospy.get_rostime()
			)

			rov.depth = depth_af
			rov.temperature = temperature_af
			rov.ex_pressure = pressure_af
	
			self.ROV_pub.publish(rov)
			rate.sleep()

def main(args):
	rospy.init_node('ms5837_pressure_sensor', anonymous=True)

	measure_depth()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)
