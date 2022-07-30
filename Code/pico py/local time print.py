import utime
import machine

while True:
    time = utime.localtime()
    print("{year:>04d}/{month:>02d}/{day:>02d}:{HH:>02d}:{MM:>02d}:{SS:>02d}".format(year=time[0], month=time[1], day=time[2],HH=time[3], MM=time[4], SS=time[5]))
    utime.sleep(1)