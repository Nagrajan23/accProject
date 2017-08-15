#!/usr/bin/env python3

#Libraries and functions needed
import smbus	#For using I2C bus
import math	
import gpsd	#For GPS Module
import datetime

#Initiliazation of I2C bus
bus = smbus.SMBus(1)

#Device Addresses
address = 0x68       # Sensor I2C address
address_mag = 0x0c  #Sensor address for magnetometer

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
usr_cntrl = 0x6a
int_pin_conf = 0x37
cntrl = 0x0a
mag_xout_l = 0x03
mag_yout_l = 0x05
mag_zout_l = 0x07

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

"""
	Accelerometer Data
"""
def acceleromter():
	# Setting power register to start getting sesnor data
	bus.write_byte_data(address, power_mgmt_1, 0)
	# Setting Acceleration register to set the sensitivity
	# 0,8,16 and 24 for 16384,8192,4096 and 2048 sensitivity respectively
	bus.write_byte_data(address, accel_config, 24)

	accel_xout = read_word_2c(accel_xout_h) #We just need to put H byte address
	accel_yout = read_word_2c(accel_yout_h) #as we are reading the word data
	accel_zout = read_word_2c(accel_zout_h)

	accel_xout_scaled = accel_xout / 2048.0 #According to the sensitivity you set
	accel_yout_scaled = accel_yout / 2048.0
	accel_zout_scaled = accel_zout / 2048.0
	return accel_xout_scaled,accel_yout_scaled,accel_zout_scaled

"""
	Gyroscope Data
"""
def gyroscope():
	# Setting power register to start getting sesnor data
	bus.write_byte_data(address, power_mgmt_1, 0)

	# Setting gyroscope register to set the sensitivity
	# 0,8,16 and 24 for 131,65.5,32.8 and 16.4 sensitivity respectively
	bus.write_byte_data(address, gyro_config, 24)
	
	gyro_xout = read_word_2c(gyro_xout_h) #We just need to put H byte address
	gyro_yout = read_word_2c(gyro_yout_h) #as we are reading the word data
	gyro_zout = read_word_2c(gyro_zout_h)

	gyro_xout_scaled = gyro_xout / 16.4 #According to the sensitivity you set
	gyro_yout_scaled = gyro_yout / 16.4
	gyro_zout_scaled = gyro_zout / 16.4
	
	return gyro_yout_scaled, gyro_yout_scaled, gyro_zout_scaled

"""
	Magnetometer Data
"""
def magnetometer():
	
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

	mag_xout_scaled = mag_xout / 0.6 #From data sheet, there is only
	mag_yout_scaled = mag_yout / 0.6 #one sensitivity level for 
	mag_zout_scaled = mag_zout / 0.6 #Magnetometer in MPU 9255
	
	return mag_xout_scaled, mag_yout_scaled, mag_zout_scaled

"""
	GPS Data
"""
def gps():
	# Connect to the local gpsd
	gpsd.connect()

	# Get gps position
	packet = gpsd.get_current()
	pos = packet.position()
	
	#Available Data from the packet
	#print(packet.position())
	#print(packet.altitude())
	#print(packet.movement())
	#print(packet.speed_vertical())
	#print(packet.speed())
	#print(packet.position_precision())
	#print(packet.map_url())
	#print(packet.time())
	return pos

"""
	Data Collection and Logging Module
"""
time = datetime.datetime.now()
pos = gps()
acc = acceleromter()
gyro = gyroscope()
mag = magnetometer()

String = "Time:" + "\t" + str(time) +"\t" + \
		 "Position:" + str(pos) +"\t" + \
		 "Accelerometer:" + str(acc) +"\t" + \
		 "Gyroscope:" + str(gyro) +"\t" + \
		 "Magnetometer:" + str(mag)

print (String)



