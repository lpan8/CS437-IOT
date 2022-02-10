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
STEP = 12
us_step = STEP
current_angle = -90
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
speed = 20


##The maximum number of coordinate points for map. Change it accordingly to the requirement
MAP_ARRAY_MAX_X = 200
MAP_ARRAY_MAX_Y = 200

#Example: Car position is at (5,0) for (10,8) map as indicated below
#

#                0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0
#Car Position->  0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0
#                0 0 0 0 0 0 0 0


angle_dist_list  = np.zeros([int(ANGLE_RANGE/STEP+1),2],dtype=int)

dist_between_points = np.zeros(int(ANGLE_RANGE/STEP),dtype=int)

cart_map = np.zeros_like(angle_dist_list)
map_x_y = np.zeros([MAP_ARRAY_MAX_X,MAP_ARRAY_MAX_Y],dtype=int)

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

def meas_dist_fill_dist_angle_bitmap(step_input):
    global angle_dist_list, current_angle, us_step
    status =0
    dist_between_points.fill(0)
    map_x_y.fill(0)
    cart_map.fill(0)


   #Scan distances for various angles using ultrasonic
    for index in range(0,step_input+1) :
        dist = int(get_distance_at(current_angle))
        temp_angle = current_angle+90
        angle_dist_list[index,0]= temp_angle
        angle_dist_list[index,1]=dist

        if ( index != step_input ):
            current_angle += us_step
        if current_angle >= max_angle:
            current_angle = max_angle
            us_step = -STEP
        elif current_angle <= min_angle:
            current_angle = min_angle
            us_step = STEP

    #Get the cartisian coordinates for each (angle,distance) point
    for k in range(angle_dist_list.shape[0]):
        if (angle_dist_list[k,1] != -2 ):
            cos_theta = np.cos(np.deg2rad(angle_dist_list[k,0]))
            cart_map[k,0] = angle_dist_list[k,1] * cos_theta

            sin_theta = np.sin(np.deg2rad(angle_dist_list[k,0]))
            cart_map[k,1] = angle_dist_list[k,1] * sin_theta

    #Calc the distances between 2 adjacent points
    for k in range(cart_map.shape[0]):
        if ( k>0 and cart_map[k-1,0] !=0 and cart_map[k-1,1] !=0 and cart_map[k,0] !=0 and cart_map[k,1] != 0):
            dist_between_points[k-1] = np.sqrt( ((cart_map[k-1,0]-cart_map[k,0])**2) + ((cart_map[k-1,1]-cart_map[k,1])**2) )

    #Algo to fill up the map using each point in the cartisian position
    # To improve: if 2 objects are apart, give the gap in the map based on the some threshold
    for j in range(dist_between_points.shape[0]):
        if dist_between_points[j] != 0 :
            for i in range(int(dist_between_points[j])):
                x = (int)( cart_map[j:j+1,0] ) + ( (i/dist_between_points[j]) * (cart_map[j+1:j+2,0] - cart_map[j:j+1,0]))
                y = (int)( cart_map[j:j+1,1] ) + ( (i/dist_between_points[j]) * (cart_map[j+1:j+2,1] - cart_map[j:j+1,1]))
                if (x < MAP_ARRAY_MAX_X/2 and y < MAP_ARRAY_MAX_Y):
                    x += MAP_ARRAY_MAX_X/2
                    xx = x.astype(int)
                    yy = y.astype(int)
                    map_x_y[xx,yy]=1


    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            print("reverse")

        final_map = map_x_y.copy()
        return final_map
    else:
        return False


def main():
    while True:
        map = meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
        print(map)
        continue

if __name__ == "__main__":
        main()

