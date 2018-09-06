import pigpio
import time

pi = pigpio.pi()
pi.bb_serial_read_close(21)
pi = pigpio.pi()
status = pi.bb_serial_read_open(21,4800,9)
print("{} \n".format(status))

pi.bb_serial_invert(21,1)

while True:
    print(pi.bb_serial_read(21))
    time.sleep(1)

