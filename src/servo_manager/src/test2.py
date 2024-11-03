from wck import servo
import time

a = servo("/dev/ttyUSB0",115200,14)

curpos = a.readPos()

print("current pos", curpos)

delta = 10
a.pos(4,curpos+delta)
time.sleep(1)
a.pos(4,curpos+delta)
