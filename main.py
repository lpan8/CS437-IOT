from point import Point
import heapq
from typing import *
import numpy as np
import picar_4wd as fc
import time
import random
import argparse
import sys
import time
from routing import Route
import enum
from object_dist_mapping import meas_dist_fill_dist_angle_bitmap

import cv2
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions

T = TypeVar('T')
power = 30
ANGLE_RANGE = 180
STEP = 12
count = 0

model = 'efficientdet_lite0.tflite'
camera_id = 0
num_threads = 4
enable_edgetpu = False
width = 640
height = 480


# Variables to calculate FPS
#counter, fps = 0, 0
start_time = time.time()

# Start capturing video input from the camera
cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, 1)

# Initialize the object detection model
options = ObjectDetectorOptions(
    num_threads=num_threads,
    score_threshold=0.3,
    max_results=3,
    enable_edgetpu=enable_edgetpu)
detector = ObjectDetector(model_path=model, options=options)

class Direction(enum.IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3



def path_to_dists(path):
    
    cur = path[0]
    dir = Direction.NORTH
    dists = [] # [(6, N), (4, E)]
    count = 0
    for i in range(1, len(path)):
        step = path[i]
        next_dir = get_next_direction(cur, step)
        if dir == next_dir:
            count += 1
            cur = step
        else:
            dists.append((count, dir))
            dir = next_dir
            count = 1
            cur = step
    dists.append((count, dir))
    return dists

def get_next_direction(cur_pos, next_pos):
    dir = Direction.NORTH
    cur_x, cur_y = cur_pos
    x, y = next_pos
    if y > cur_y and cur_x == x:
        dir = Direction.EAST
    elif cur_x < x and cur_y == y:
        dir = Direction.NORTH
    elif cur_x > x and cur_y == y:
        dir = Direction.SOUTH
    elif cur_y > y and cur_x == x:
        dir = Direction.WEST
    return dir

def dist_to_time(dist):
    global count
    global cap
    global detector

    stop = False
    object_detected = False

    speed = fc.Speed(25)
    speed.start()
    fc.forward(power)
    x = 0
    us = fc.us
    while x <= (dist * 0.4):
        dist = us.get_distance()
        print(dist)
        if dist <= 2:
            fc.stop()
            #fc.backward(50)
            #time.sleep(0.5)
            object_detected = True
            break
            
        # Run object detection estimation using the model.
        success, image = cap.read()
        if not success:
            sys.exit(
                'ERROR: Unable to read from webcam. Please verify your webcam settings.'
            )
        image = cv2.flip(image, 1)
        detections = detector.detect(image)
        for detection in detections:
            category = detection.categories[0]
            class_name = category.label
            print(class_name)
            if class_name == 'stop sign':
                stop = True

        if stop == True and count < 1:
            print("saw stop sign")
            fc.stop()
            time.sleep(2)
            stop = False
            count += 1
            fc.forward(power)
        time.sleep(0.1)
        cur_speed = speed() 
        x += cur_speed * 0.1
        #print("%smm/s"%cur_speed)
    #print("%smm"%x)
    speed.deinit()
    fc.stop()
    return object_detected, x

def turn_90_right():
    fc.turn_right(100)
    time.sleep(0.8)


def turn_90_left():
    fc.turn_left(100)
    time.sleep(0.95)

def get_path(map, start, end):
    new_map = np.full(np.shape(map), -1)
    came_from, cost_so_far = Route.search(map, start, end)
    for point, cost in cost_so_far.items():
        new_map[point.y][point.x] = cost

    # for point in came_from:
    #     point.print()

    
    path = []
    last_point = end
    #print(last_point)
    #print(last_point in came_from)
    path.append((last_point.x, last_point.y))
    while last_point is not None:
        #print(last_point)
        last_point = came_from[last_point]
        if last_point is not None:
            path.append((last_point.x, last_point.y))
    path.reverse()
    return path

def is_within_dest(p1, p2):
    if abs(p1.x - p2.x) < 10 and abs(p1.y - p2.y) < 10:
        return True
    else:   
        return False

def main():

    map = meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
    #print(map)

    start = Point(0, 100)  # starting position
    end = Point(150, 110)  # ending position

    # start = Point(0, 100)  # starting position
    # end = Point(140, 150)  # ending position

    cur_pos = start
    change_x = 0
    change_y = 0
    total_x = 0
    total_y = 0
    while not is_within_dest(cur_pos, end):
        path = get_path(map, start, end)
        #print(path)
        dir_dists = path_to_dists(path)
        print(dir_dists)
        car_direction = Direction.NORTH

        for dist, dir in dir_dists:

            dir_diff = dir - car_direction
            print(dir, car_direction, dir_diff)
            print("cur pos: ", cur_pos)
            print("dest: ", end)
            if dir_diff % 4 == 3:
                turn_90_left()
                car_direction = dir 
                print("turned left")
                if car_direction == Direction.NORTH:
                    # rescan
                    #print(map)
                    fc.stop()
                    map = meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
                    print("facing north again")
                    end = Point(end.x - total_x, end.y - total_y)
                    cur_pos = start
                    # cur_pos = Point(cur_pos.x + change_x, cur_pos.y + change_y)
                    total_x, total_y = 0, 0
                   
                   # print("new dest: ", end)
                    continue

            elif dir_diff % 4 == 1:
                turn_90_right()
                car_direction = dir
                print("turned right")
                if car_direction == Direction.NORTH:
                    # rescan
                    fc.stop()
                    print("facing north again")
                    map = meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
                    end = Point(end.x - total_x, end.y - total_y)
                    cur_pos = start
                    total_x, total_y = 0, 0
                   # print("cur pos: ", cur_pos)
                    #print("new dest: ", end)
                    continue
                    
            object_detected, travelled = dist_to_time(dist)
            if (object_detected):
                dist = travelled / 10
            if dir == Direction.NORTH:
                change_x += dist
            elif dir == Direction.EAST:
                change_y += dist
            elif dir == Direction.SOUTH:
                change_x -= dist
            elif dir == Direction.WEST:
                change_y -= dist
            print(change_x, change_y)
            cur_pos = Point(cur_pos.x + change_x, cur_pos.y + change_y)
            total_x += change_x
            total_y += change_y
            change_x, change_y = 0, 0
            if (object_detected):
                map = meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
                end = Point(end.x - total_x, end.y - total_y)
                cur_pos = start
                total_x, total_y = 0, 0
            
            
    
        
    
if __name__ == '__main__':
    try:
        main()
    finally:
        fc.stop()
# [(6, N), (4, E), (7, )]
