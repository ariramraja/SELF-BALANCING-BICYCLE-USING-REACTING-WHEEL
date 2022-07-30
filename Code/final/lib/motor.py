from machine import Pin, PWM
from time import sleep

# pin declarations
    # pins for motor 1
en1 = Pin(8, value = 0, mode=Pin.OUT)
cw = Pin(7, value=0, mode=Pin.OUT) 
acw =Pin(9, value=0, mode=Pin.OUT)

    # pins for motor 2
en2 = Pin(20, value=0, mode=Pin.OUT)
in2a = Pin(19, value=0, mode=Pin.OUT)
in2b = Pin(21, value=0, mode=Pin.OUT)
#servo value set
MID= 1300000
MIN = 900000
MAX= 1900000

#servo pin for PWM
pwm = PWM (Pin (15))
Speed1=PWM(en1)
speed=PWM(en2)

def backward(speed_):
    Speed1.freq(50)
    Speed1.duty_u16(int(speed_/100*65536))
    cw.value(0)
    acw.value(1)
def forward(speed_):
    Speed1.freq(50)
    Speed1.duty_u16(int(speed_/100*65536))
    cw.value(1)
    acw.value(0)
def stop():
    cw.value(0)
    acw.value(0)
def up():
    speed.duty_u16(10000)
    speed.freq(50)
    in2a(1)
    in2b(0)
def down():
    speed.duty_u16(10000)
    speed.freq(50)
    in2a(0)
    in2b(1)
def stop_():
    in2a(0)
    in2b(0)
def servo_left():
    pwm.freq(50)
    pwm.duty_ns (50)
    pwm.duty_ns (MIN)
    
# Backward
def servo_forward():
    pwm.freq(50)
    pwm.duty_ns (50)
    pwm.duty_ns (MID)
#Turn Right
def servo_right():
    pwm.freq(50)
    pwm.duty_ns (50)
    pwm.duty_ns (MAX)