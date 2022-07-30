import math, utime, time
from time import sleep
from machine import Pin, I2C, PWM, UART
from imu import MPU6050
from PID import PID
import motor as MOTOR

#measuring angle x,y &z
gyro_scale = 131.0
accel_scale = 16384.0
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846

#onboad led
led_onboard = machine.Pin(25, machine.Pin.OUT)
led_onboard.value(1)
#Gyeroscop sensor
i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=500000)
imu = MPU6050(i2c)

#Defining UART channel and Baud Rate
uart= UART(0,9600)

data =2

K = 0.98
K1 = 1 - K

time_diff = 0.1

#sensor = MPU6050(bus, address, "MPU6050")
#sensor.read_raw_data()	# Reads current data from the sensor

rate_gyroX = 0.0
rate_gyroY = 0.0
rate_gyroZ = 0.0

gyroAngleX = 0.0 
gyroAngleY = 0.0 
gyroAngleZ = 0.0 

raw_accX = 0.0
raw_accY = 0.0
raw_accZ = 0.0

rate_accX = 0.0
rate_accY = 0.0
rate_accZ = 0.0

pitch = 0.0
roll = 0.0

accAngX = 0.0

CFangleX = 0.0
CFangleX1 = 0.0

K = 0.98

FIX = -12.89

'''def pitch_ (pitch): 
    pitch = 180 * math.atan2(imu.accel.x, math.sqrt(imu.accel.y*imu.accel.y + imu.accel.z*imu.accel.z))/M_PI
    return pitch

while True:
    print "{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format( time.time() - now, (last_x), gyro_total_x, (last_x), (last_y), gyro_total_y, (last_y))
    time.sleep(0.1)'''
def dist(a, b):
    return math.sqrt((a * a) + (b * b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
p=PID(1.0,-0.04,0.0)
p.setPoint(0.0)

for i in range(0, int(300.0 / time_diff)):
    time.sleep(time_diff - 0.005)
    
    #chang
    # Gyroscope value Degree Per Second / Scalled Data
    
    rate_gyroX = imu.gyro.x
    rate_gyroY = imu.gyro.y
    rate_gyroZ = imu.gyro.z
    # The angle of the Gyroscope
    gyroAngleX += rate_gyroX * time_diff 
    gyroAngleY += rate_gyroY * time_diff 
    gyroAngleZ += rate_gyroZ * time_diff 
    
    # Accelerometer Raw Value
    raw_accX = imu.accel.x
    raw_accY = imu.accel.y
    raw_accZ = imu.accel.z
    
    # Accelerometer value Degree Per Second / Scalled Data
    rate_accX = imu.accel.x
    rate_accY = imu.accel.y
    rate_accZ = imu.accel.z
    
    # http://ozzmaker.com/2013/04/18/success-with-a-balancing-robot-using-a-raspberry-pi/
    accAngX = ( math.atan2(rate_accX, rate_accY) + M_PI ) * RAD_TO_DEG
    CFangleX = K * ( CFangleX + rate_gyroX * time_diff) + (1 - K) * accAngX
    
    # http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html 
    accAngX1 = get_x_rotation(rate_accX, rate_accY, rate_accX)
    CFangleX1 = ( K * ( CFangleX1 + rate_gyroX * time_diff) + (1 - K) * accAngX1 )
    
    # Followed the Second example because it gives resonable pid reading
    pid =int(p.update(CFangleX1))  
    speed_ = pid * 30
    print(data,pid,"       ",end="\r")

    if uart.any(): #Checking if data available in bluetooth
        data = uart.read() #Getting data
        data = int(data)
        
    if(pid > 0):
        MOTOR.forward(speed_)
    if(pid < 0):
        MOTOR.backward(abs(speed_))
    if(data == 1):  
        MOTOR.servo_left()
        sleep(1.5)
        data=2
    if(data == 2):
        MOTOR.servo_forward()
    if(data == 3):
        MOTOR.servo_right()
        sleep(1.5)
        data=2
    if(data == 4):
        MOTOR.up()
    if(data == 5):
        MOTOR.down()
    if(data == 6):
        MOTOR.stop_()
  
