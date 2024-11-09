from wck import servo
import time

a = servo("/dev/ttyUSB0",115200)

curpos = a.readPos(14)

print("current pos", curpos)

delta = 15
a.pos(14, 4,curpos+delta)
time.sleep(1)
a.pos(14, 4,curpos-delta)
time.sleep(1)

a.pos(14, 4,curpos)
time.sleep(1)
a.close()