from __future__ import division
import time
import math
import csv

import RPi.GPIO as GPIO

from gpiozero import DistanceSensor
import time
import csv

ratio=0.95 #ratio motor1 to motor2

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

LED = 7
Taster = 5

#Linesensors
cny70_left = 16
cny70_right = 26

#SR04
trigger=25
echo=27

Motor1_PWM = 18
Motor1_IN1 = 17
Motor1_IN2 = 22

Motor2_PWM = 19
Motor2_IN1 = 24
Motor2_IN2 = 4

def M1_forward():
    GPIO.output(Motor1_IN2,GPIO.LOW)
    GPIO.output(Motor1_IN1,GPIO.HIGH)
    
def M1_backward():
    GPIO.output(Motor1_IN1,GPIO.LOW)
    GPIO.output(Motor1_IN2,GPIO.HIGH)

def M2_forward():
    GPIO.output(Motor2_IN2,GPIO.LOW)
    GPIO.output(Motor2_IN1,GPIO.HIGH)
    
def M2_backward():
    GPIO.output(Motor2_IN1,GPIO.LOW)
    GPIO.output(Motor2_IN2,GPIO.HIGH)

GPIO.setup(Motor1_IN1,GPIO.OUT)
GPIO.setup(Motor1_IN2,GPIO.OUT)
GPIO.setup(Motor1_PWM,GPIO.OUT)
PWM_1 = GPIO.PWM(Motor1_PWM, 90) #GPIO als PWM mit Frequenz 90Hz
PWM_1.start(0) #Duty Cycle = 0

GPIO.setup(Motor2_IN1,GPIO.OUT)
GPIO.setup(Motor2_IN2,GPIO.OUT)
GPIO.setup(Motor2_PWM,GPIO.OUT)
PWM_2 = GPIO.PWM(Motor2_PWM, 90) #GPIO als PWM mit Frequenz 90Hz
PWM_2.start(0) #Duty Cycle = 0

GPIO.setup(Taster, GPIO.IN, pull_up_down = GPIO.PUD_UP)
time_init=time.time()
sensor = DistanceSensor(echo=27, trigger=25,max_distance=2)
array=[]

#1:foward | 2:backward | 3:left | 4:right
def linear_dir(dir):
    if(dir==1):
        M1_forward()
        M2_forward()
    elif(dir==2):
        M1_backward()
        M2_backward()
    elif(dir==3):
        M1_backward()
        M2_forward()
    elif(dir==4):
        M1_forward()
        M2_backward()
    else:
        GPIO.output(Motor1_IN1,GPIO.LOW) 
        GPIO.output(Motor1_IN2,GPIO.LOW) 
        GPIO.output(Motor2_IN1,GPIO.LOW) 
        GPIO.output(Motor2_IN2,GPIO.LOW) 
        # print("stop!!")

max_range=2
# odom transfer directly to map coordinate
class Odom(object):
    def __init__(self):
        self.x=[]
        self.y=[]
        self.theta=[]
        self.pose=[0,0,0]
        self.x.append(0)
        self.y.append(0)
        self.theta.append(0)
    def update_pose(self):
        self.pose=[self.x[len(self.x)-1],self.y[len(self.y)-1],self.theta[len(self.theta)-1]]
    def update_odom(self,speed,t):
        if(speed.x!=0 and speed.theta==0):
            # linear movement -> only update x pos
            dis=speed.x*t
            self.x.append(self.pose[0]+dis*math.cos(self.pose[2]))
            self.y.append(self.pose[1]+dis*math.sin(self.pose[2]))
            self.theta.append(self.pose[2])
            self.update_pose()
        if(speed.theta!=0 and speed.x==0):
            # rotation movement -> only update theta ori
            # speed.theta [m/s]-> [rad/s]: speed.theta/cir(0.393)/2pi
            dis=speed.theta*t/(0.393*2*math.pi)
            self.x.append(self.pose[0])
            self.y.append(self.pose[1])
            self.theta.append(self.pose[2]+dis)
            self.update_pose()

# map_resolution=0.01[m]->round up/down every fraction factor
# max range=2[m]
# if there is movement then run update map
# offset from pose to the data -> status=1
# the data point(if object) -> status=2
# from data point to boundary -> status=0 (this need some addition tests)
class Map(object):
    def __init__(self):
        self.x=[]
        self.y=[]
        self.status=[]
    def update_map(self,scan_data):
        map_x=math.cos(odom.pose[2])*scan_data+odom.pose[0]
        map_y=math.sin(odom.pose[2])*scan_data+odom.pose[1]
        spare_data=fill_map(scan_data)
        for i in range(len(spare_data[1])):
            self.x.append(spare_data[0][i])
            self.y.append(spare_data[1][i])
            self.status.append(1)
        if(scan_data==max_range):
            self.x.append(map_x)
            self.y.append(map_y)
            self.status.append(1)
        else:
            self.x.append(map_x)
            self.y.append(map_y)
            self.status.append(2)

# with map_resolution=0.01[m] -> number of points between origin and the data point is the scan
# or calculate with: scan_data/map_resolution
def fill_map(scan_data):
    unit_vector=[]
    map_x=[]
    map_y=[]
    temp=[]
    unit_vector.append((math.cos(odom.pose[2])*scan_data-odom.pose[0])/scan_data)
    unit_vector.append((math.sin(odom.pose[2])*scan_data-odom.pose[1])/scan_data)
    for i in range(int(scan_data)):
        map_x.append(unit_vector[0]*i)
        map_y.append(unit_vector[1]*i)
    temp.append(map_x)
    temp.append(map_y)
    return temp

class Speed():
    def __init__(self,v_x,v_theta):
        self.x=v_x
        self.theta=v_theta

# convert speed of robot [m/s] <-> speed of motor [% duty cycle]
# max duty cycle(100) ~= 0.65[m/s]
def convert_speed(duty_2,dir):
    convert_ratio=0.65
    if (dir==1 or dir==2):
        speed.x=duty_2*convert_ratio
    elif (dir==3 or dir==4):
        speed.theta=duty_2*convert_ratio

map=Map()
odom =Odom()
speed=Speed(0,0)
while 1:
    # odom.update_odom(speed,time.time()-time_init)
    # time_init=time.time()
    # if(odom.x[len(odom.x)-1]<1 and odom.theta[len(odom.theta)-1]<1):
    #     speed.x=1
    #     time.sleep(0.1)
    # elif(odom.x[len(odom.x)-1]>1 and odom.theta[len(odom.theta)-1]<1):
    #     speed.x=0
    #     speed.theta=1
    #     time.sleep(0.1)
    # elif (odom.x[len(odom.x)-1]>1 and odom.theta[len(odom.theta)-1]>1):
    #     speed.x=0
    #     speed.theta=0
    #     break
    if GPIO.input(Taster)==GPIO.LOW:
        time_init= time.time()
        duty=100
        dir=3
        linear_dir(dir) #set dir
        PWM_1.ChangeDutyCycle(duty*ratio) 
        PWM_2.ChangeDutyCycle(duty)
        convert_speed(duty,dir)
        for i in range(40):
            odom.update_odom(speed,time.time()-time_init)
            map.update_map(sensor.distance * 100)
            time_init=time.time()
            time.sleep(0.05)
        linear_dir(0)
        break
# write csv file syntax
data=open("test.csv","w")
csv_write=csv.writer(data)
csv_write.writerow(odom.x)
csv_write.writerow(odom.y)
csv_write.writerow(odom.theta)
csv_write.writerow(map.x)
csv_write.writerow(map.y)
csv_write.writerow(map.status)
data.close()
# print(len(map.status))
# print(len(odom.theta))
