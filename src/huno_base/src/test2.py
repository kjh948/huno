from wck import servo
import time

a = servo("/dev/tty.usbserial-14440",115200)
a.scan()

id = 15
curpos = a.readPos(id)

print("current pos", curpos)

delta = 5

a.pos(id, 4,curpos+delta)
time.sleep(1)
a.pos(id, 4,curpos-delta)
time.sleep(1)

a.pos(id, 4,curpos)
time.sleep(1)
a.close()