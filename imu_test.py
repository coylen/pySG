from MPU9250.mpu9250 import MPU9250
import time
from math import asin, cos, sin, atan, degrees, pi, sqrt
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

_mag_bias = (45.746011, -1.976222, 30.256430)
_mag_scale = ((0.817553, -0.025437, -0.005537),
                  (-0.025437, 0.820258, 0.012555),
                  (-0.005537, 0.012555, 0.802883))


bob = MPU9250(0)

def getmag():
    mag=bob.mag.xyz
    mx_raw = mag[0] - _mag_bias[0]
    my_raw = mag[1] - _mag_bias[1]
    mz_raw = mag[2] - _mag_bias[2]

    mx = mx_raw*_mag_scale[0][0]+my_raw*_mag_scale[0][1]+mz_raw*_mag_scale[0][2]
    my = mx_raw*_mag_scale[1][0]+my_raw*_mag_scale[1][1]+mz_raw*_mag_scale[1][2]
    mz = mx_raw*_mag_scale[2][0]+my_raw*_mag_scale[2][1]+mz_raw*_mag_scale[2][2]

    return (mx,my,mz)

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

#gyrocal()

#testy = Fusion(timediff=piDeltaTime)
count =0
#print("cali")
#testy.calibrate(getmag, stt)

while True:
    #testy.update(bob.accel.xyz,(0,0,0), bob.mag.xyz, time.perf_counter())#
    with open("/share/code/silvergirl/mag.dat", mode='a') as f:

        if count % 50 ==0:
            tr=getmag()
            aww="{:7.3f} {:7.3f} {:7.3f} \n".format(tr[0], tr[1], tr[2])
            w=f.write(aww)
           # print(w)
            a=bob.accel.xyz
            a_s = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])
            a1=[0.0,0.0,0.0]
            a1[0] = a[0] / a_s
            a1[1] = a[1] / a_s
            a1[2] = a[2] / a_s
      #      print(a1)
            ptch=asin(-a1[0])
            if ptch!= pi/2 and ptch!=-pi/2:
                roll=asin(-a1[1]/cos(ptch))
            else:
                roll = 0
            mx2 = tr[0]*cos(ptch)+tr[2]*sin(ptch)
            my2 =  tr[0]*sin(roll)*sin(ptch)+tr[1]*cos(roll)-tr[2]*sin(roll)*cos(ptch)
            mz2 = -tr[0]*cos(roll)*sin(ptch)+tr[1]*sin(roll)+tr[2]*cos(roll)*cos(ptch)
            if mx2>0 and my2>=0:
                heading = atan(my2/mx2)
            elif mx2<0:
                heading = pi+atan(my2/mx2)
            elif mx2>0 and my2<0:
                heading = 2*pi+atan(my2/mx2)
            elif mx2==0 and my2<0:
                heading = pi/2
            else:
                heading = 1.5*pi

 #           print("accel {:7.3f}".format( a1[0]*a1[0]+a1[1]*a1[1]+a1[2]*a1[2]))
            print("pitch {:7.3f} roll {:7.3f} heading {:7.3f}".format(degrees(ptch), degrees(roll), degrees(heading)))

        count+=1


