from wck import servo
import time

a = servo("/dev/ttyUSB0",115200,14)

curpos = a.readPos()

print("current pos", curpos[0])

delta = 30
a.pos(4,curpos[0]+delta)
time.sleep(1)
a.pos(4,curpos[0]-delta)
time.sleep(1)

a.pos(4,curpos[0])
time.sleep(1)
a.close()