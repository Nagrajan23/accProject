#!/usr/bin/env python3

#Libraries and functions needed
import smbus	#For using I2C bus
import math	
#import gpsd	#For GPS Module
import datetime
#from time import sleep
#import RPi.GPIO as GPIO

#Initiliazation of I2C bus
bus = smbus.SMBus(1)

#Status LED initiliazation
#GPIO.setmode(GPIO.BCM)
#LIGHT_PIN = 25

#Device Addresses
address = 0x68       # Sensor I2C address
#address_mag = 0x0c  #Sensor address for magnetometer

#Control Addresses
power_mgmt_1 = 0x6b

# Register address from MPU 9255 register map for Accelerometer
accel_config = 0x1c
accel_xout_h = 0x3b
accel_yout_h = 0x3d
accel_zout_h = 0x3f

# Register address from MPU 9255 register map for Gyroscope
gyro_config = 0x1b
gyro_xout_h = 0x43
gyro_yout_h = 0x45
gyro_zout_h = 0x47

# Register address from MPU 9255 register map for Magnetometer
#usr_cntrl = 0x6a
#int_pin_conf = 0x37
#cntrl = 0x0a
#mag_xout_l = 0x03
#mag_yout_l = 0x05
#mag_zout_l = 0x07

"""
	Common Functions to be used
"""

def read_byte(address, adr):
	return bus.read_byte_data(address, adr)

def read_word(adr):
	high = bus.read_byte_data(address, adr)
	low = bus.read_byte_data(address, adr+1)
	val = (high << 8) + low
	return val

def read_word_2c(adr):
	val = read_word(adr)
	if (val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val
'''
def read_word_mag(address, adr):
	low = read_byte(address, adr)
	high = read_byte(address, adr+1)
	val = (high << 8) + low
	return val

def read_word_2c_mag(address, adr):
	val = read_word_mag(address, adr)
	if (val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val
'''
"""
	Accelerometer Data
"""
def acceleromter():
	try:
		accel_xout = read_word_2c(accel_xout_h) #We just need to put H byte address
		accel_yout = read_word_2c(accel_yout_h) #as we are reading the word data
		accel_zout = read_word_2c(accel_zout_h)

		accel_xout_scaled = accel_xout / 8192.0 #According to the sensitivity you set
		accel_yout_scaled = accel_yout / 8192.0
		accel_zout_scaled = accel_zout / 8192.0
		return accel_xout_scaled,accel_yout_scaled,accel_zout_scaled
	except Exception as e:
		return 'accel_xout_scaled','accel_yout_scaled','accel_zout_scaled'

"""
	Gyroscope Data
"""
def gyroscope():
	try:
		gyro_xout = read_word_2c(gyro_xout_h) #We just need to put H byte address
		gyro_yout = read_word_2c(gyro_yout_h) #as we are reading the word data
		gyro_zout = read_word_2c(gyro_zout_h)

		gyro_xout_scaled = gyro_xout / 65.5 #According to the sensitivity you set
		gyro_yout_scaled = gyro_yout / 65.5
		gyro_zout_scaled = gyro_zout / 65.5
		
		return gyro_xout_scaled, gyro_yout_scaled, gyro_zout_scaled
	except Exception as e:
		return 'gyro_xout_scaled', 'gyro_yout_scaled', 'gyro_zout_scaled'

"""
	Magnetometer Data
"""
'''
def magnetometer():
	try:
				
		#initialize the power registers to wake up sensor
		bus.write_byte_data(address, power_mgmt_1, 0)
		
		#disable master i2c mode of sensor
		bus.write_byte_data(address, usr_cntrl, 0)
		
		# enable bypass mode to read directly from magnetometer
		bus.write_byte_data(address, int_pin_conf, 2)
		
		# setup magnetic sensors for contiuously reading data
		bus.write_byte_data(address_mag, 0x0a, 18)
		
		mag_xout = read_word_2c_mag(address_mag, mag_xout_l)
		mag_yout = read_word_2c_mag(address_mag, mag_yout_l)
		mag_zout = read_word_2c_mag(address_mag, mag_zout_l)

		mag_xout_scaled = mag_xout * 0.6 #From data sheet, there is only
		mag_yout_scaled = mag_yout * 0.6 #one sensitivity level for 
		mag_zout_scaled = mag_zout * 0.6 #Magnetometer in MPU 9255 (Multiply as described in datasheet)
		
		return mag_xout_scaled, mag_yout_scaled, mag_zout_scaled
	except Exception as e:
		return 'mag_xout_scaled', 'mag_yout_scaled', 'mag_zout_scaled'

"""
	GPS Data
"""
def gps():
	try:
			
		# Connect to the local gpsd
		gpsd.connect()

		# Get gps position
		packet = gpsd.get_current()
		#pos = packet.position()
		[lat, lng ]=packet.position()
		
		#Available Data from the packet
		#print(packet.position())
		#print(packet.altitude())
		#print(packet.movement())
		#print(packet.speed_vertical())
		#print(packet.speed())
		#print(packet.position_precision())
		#print(packet.map_url())
		#print(packet.time())
		return lat, lng
	except	Exception as e:
		return 'lat', 'lng'
'''
"""
	Data Collection and Logging Module
"""
String_head = "Time" + "\t" + \
		 "Position_lat" +"\t" + \
		 "Position_lng" +"\t" + \
		 "Accelerometer_x" +"\t" + \
		 "Accelerometer_y" +"\t" + \
		 "Accelerometer_z" +"\t" + \
		 "Gyroscope_x" +"\t" + \
		 "Gyroscope_y" +"\t" + \
		 "Gyroscope_z" +"\t" + \
		 "Magnetometer_x" + '\t' + \
		 "Magnetometer_y" + '\t' + \
		 "Magnetometer_z" + '\n'

	
# Setting power register to start getting sesnor data
bus.write_byte_data(address, power_mgmt_1, 0)
# Setting Acceleration register to set the sensitivity
# 0,8,16 and 24 for 16384,8192,4096 and 2048 sensitivity respectively
bus.write_byte_data(address, accel_config, 8) # for +/- 4g
# Setting gyroscope register to set the sensitivity
# 0,8,16 and 24 for 131,65.5,32.8 and 16.4 sensitivity respectively
bus.write_byte_data(address, gyro_config, 8)

datafile = datetime.datetime.now().strftime("%Y%m%d-%H%M")
e = open(datafile + '.txt','a',)
e.write(String_head)
print ("Start Getting data")
while True:
	try:
		#PIN = 25
		#GPIO.setup(PIN, GPIO.OUT)
		#GPIO.output(PIN, False)	
		time = datetime.datetime.now()
		#[lat, lng] = gps()
		[Accelerometer_x, Accelerometer_y, Accelerometer_z] = acceleromter()
		[Gyroscope_x, Gyroscope_y, Gyroscope_z] = gyroscope()
		#sleep(0.005)	#Checking Magnetometer response if wait
		#[Magnetometer_x, Magnetometer_y, Magnetometer_z] = magnetometer()

		'''String = "Time:" + "\t" + str(time) +"\t" + \
				 "Position:" + str(pos) +"\t" + \
				 "Accelerometer:" + str(acc) +"\t" + \
				 "Gyroscope:" + str(gyro) +"\t" + \
				 "Magnetometer:" + str(mag) + '\n'
		'''
		#ttime = datetime.datetime.now().strftime("%H:%M:%S.%f")
		#print datetime.datetime.now()
		#ttime = datetime.date(2002, 12, 26).strftime("%Y-%m-%d %H:%M:%S.%f")
		time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
		String_body = str(time) + "\t" + \
			 "lat" +"\t" + \
			 "lng" +"\t" + \
			 str(Accelerometer_x) +"\t" + \
			 str(Accelerometer_y) +"\t" + \
			 str(Accelerometer_z) +"\t" + \
			 str(Gyroscope_x) +"\t" + \
			 str(Gyroscope_y) +"\t" + \
			 str(Gyroscope_z) +"\t" + \
			 "Magnetometer_x" + '\t' + \
			 "Magnetometer_y" + '\t' + \
			 "Magnetometer_z" + '\n'
		e.write(String_body)
		#GPIO.setup(PIN, GPIO.OUT)
		#GPIO.output(PIN, True)
		print ('Data is being recorded: '+ str (time))
		#sleep(0.005)	#Wait for next iteration
		#print (String)
		#sleep(2)
	except KeyboardInterrupt:
		e.close()
		exit()
