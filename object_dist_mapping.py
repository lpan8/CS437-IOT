import picar_4wd as fc
from picar_4wd.pwm import PWM
#from picar_4wd.adc import ADC
from picar_4wd.pin import Pin
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.utils import *
import time
import numpy as np
import sys


ANGLE_RANGE = 180

#STEP indicates the angle steps
STEP = 12
us_step = STEP
#curr angle -90 indicates right most position of car
current_angle = -90
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2

#DS to store the measured angle and distance points
angle_dist_list  = np.zeros([int(ANGLE_RANGE/STEP+1),2],dtype=int)

MAX_DISTANCE = 100
#map_array: 2D grid for distance and angle bitmap for one sweep for 180 degrees.
#Row: indicates distance in cm
#Column: indicates angle from 0 to 179
map_array = np.zeros([MAX_DISTANCE,180],dtype=int)

servo = Servo(PWM("P0"), offset=0)

us = Ultrasonic(Pin('D8'), Pin('D9'))

np.set_printoptions(threshold=sys.maxsize)

def get_distance_at(angle):
    global angle_distance
    servo.set_angle(angle)
    time.sleep(0.04)
    distance = us.get_distance()
    angle_distance = [angle, distance]
    return distance


#The below method measures distacne for various angles and fills up bitmap for distance and angle in 2D grid.
def meas_dist_fill_dist_angle_bitmap(step_input):
    global angle_dist_list, current_angle, us_step
    status =0
    map_array.fill(0)
    for index in range(0,step_input+1) :
        dist = int(get_distance_at(current_angle))
        temp_angle = current_angle+90
        angle_dist_list[index,0]= temp_angle
        angle_dist_list[index,1]=dist
        if ( index > 0 ) and ( dist < MAX_DISTANCE ) and ( angle_dist_list[index,1] > 0 ) and ( abs (angle_dist_list[index,1]-angle_dist_list[index-1,1]) <= 2.0 ):
            if  angle_dist_list[index-1,0] < angle_dist_list[index,0] :
                map_array[angle_dist_list[index,1],angle_dist_list[index-1,0]:angle_dist_list[index,0]]=1
            else:
                map_array[angle_dist_list[index,1],angle_dist_list[index,0]:angle_dist_list[index-1,0]]=1

        if ( index != step_input ):
            current_angle += us_step
        if current_angle >= max_angle:
            current_angle = max_angle
            us_step = -STEP
        elif current_angle <= min_angle:
            current_angle = min_angle
            us_step = STEP

    if current_angle == min_angle or current_angle == max_angle:
        tmp = map_array.copy()
        return tmp
    else:
        return False

def main():
    while True:
        map_array_l = meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
        #print(map_array_l)


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()

