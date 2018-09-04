from MPU9250.mpu9250 import MPU9250
import time
from Fusion.fusion import Fusion
# use time.perf_counter_ns() for timestamaps
# Deltatime function converts to the expected microsecond format
def piDeltaTime(end, start):
    return (end-start)*1e6
a=time.time()
def stt():
    if(time.time()-a)>60:
        return True
    return False



bob = MPU9250(0)

def getmag():
    return bob.mag.xyz

def gyrocal():
    xa = 0
    ya = 0
    za = 0
    for x in range(0, 100):
        xyz = bob.accel.xyz
        xa += xyz[0]
        ya += xyz[1]
        za += xyz[2]
        time.sleep(0.1)
    print(xa/100, ya/100, za/100)

gyrocal()

testy = Fusion(timediff=piDeltaTime)
count =0
#print("cali")
#testy.calibrate(getmag, stt)

while True:
    testy.update(bob.accel.xyz,(0,0,0), bob.mag.xyz, time.perf_counter())#

    if count % 50 ==0:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(testy.heading, testy.pitch, testy.roll))
        #print(bob.mag.xyz)
    time.sleep(0.02)
    count+=1


