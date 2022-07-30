from machine import I2C, Pin
from urtc import DS1307
import utime

i2c = I2C(0,scl = Pin(1),sda = Pin(0),freq = 400000)
rtc = DS1307(i2c)
'''
year = int(input("Year : "))
month = int(input("month (Jan --> 1 , Dec --> 12): "))
date = int(input("date : "))
day = int(input("day (1 --> monday , 2 --> Tuesday ... 0 --> Sunday): "))
hour = int(input("hour (24 Hour format): "))
minute = int(input("minute : "))
second = int(input("second : "))

now = (year,month,date,day,hour,minute,second)
rtc.datetime(now)'''

while True:
    (year,month,date,day,hour,minute,second)=rtc.datetime()
    utime.sleep(1)
    time = rtc.datetime()
    #print(rtc.datetime())
    print("{year:>04d}/{month:>02d}/{day:>02d}\n{HH:>02d}:{MM:>02d}:{SS:>02d}".format(year=time[0], month=time[1], day=time[2],HH=time[4], MM=time[5], SS=time[6] ))