# More than 30,000 datapoints can be stored.
# Practical example: Saving data at every minute will give
# Approx Backup time = 30000/1/60/24 = approx 20 days of backup
from machine import Pin, I2C
from DHT22 import DHT22
from urtc import DS3231
import time

# Initializing I2C
i2crtc = I2C(0, scl=Pin(21), sda=Pin(20), freq=100000)
exRTC = DS3231(i2crtc)

# Getting Date and Time from RTC
pDT = exRTC.datetime()
print("Present Date & Time :", pDT)

# Iitializing DHT22 Pin
dht22 = DHT22(Pin(15, Pin.IN, Pin.PULL_UP))

while True:
    # Reading Temperature and Humidty from DHT22
    T, H = dht22.read()

    # Getting Date and Time from RTC
    pDT = exRTC.datetime()
    print(pDT[0], pDT[1], pDT[2], pDT[3], pDT[4], pDT[5], pDT[6], T, H)

    # Backing up data into internal storage in the below syntax
    # year, month, day, weekday, hour, minute, second, temperature, humidity
    with open('Databackup.txt', 'a+') as file:
        file.write(str(pDT[0]) + ", " + str(pDT[1]) + ", " + str(pDT[2]) + ", " + str(pDT[3]) + ", " +
                   str(pDT[4]) + ", " + str(pDT[5]) + ", " + str(pDT[6]) + ", " + str(T) + ", " + str(H))
        file.write('\n')
        file.close()

    # Since DHT22 updates temperature and humidity every 2 seconds
    time.sleep(2)