from wck import servo

a = servo("/dev/ttyUSB0",115200,0)
a.scan()
