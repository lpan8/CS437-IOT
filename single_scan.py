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

T = TypeVar('T')
power = 30
ANGLE_RANGE = 180
STEP = 12

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
    speed = fc.Speed(25)
    speed.start()
    fc.forward(power)
    x = 0
    while x <= (dist * 0.6):
        time.sleep(0.1)
        cur_speed = speed() 
        x += cur_speed * 0.1
        #print("%smm/s"%cur_speed)
    print("%smm"%x)
    speed.deinit()
    fc.stop()

def turn_90_right():
    fc.turn_right(100)
    time.sleep(0.8)


def turn_90_left():
    fc.turn_left(100)
    time.sleep(0.4)


def main():

    # # map = np.array([[0, 1, 1, 1, 1, 1],
    # #                  [0, 0, 0, 0, 0, 0],
    # #                  [0, 1, 1, 1, 0, 0],
    # #                  [0, 1, 0, 1, 1, 0],
    # #                  [0, 1, 0, 1, 1, 0]])

    
    # map = np.array([[0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    map = meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
    print(map)
    new_map = np.full(np.shape(map), -1)

    # start = Point(50 , 0)  # starting position
    # end = Point(50, 99)  # ending position

    start = Point(0 , 50)  # starting position
    end = Point(80, 50)  # ending position


    # goal dest: 

    came_from, cost_so_far = Route.search(map, start, end)
    for point, cost in cost_so_far.items():
        new_map[point.y][point.x] = cost

    path = []

    #print(new_map)
    last_point = end
    #print(last_point)

    path.append((last_point.x, last_point.y))

    #print(came_from)

    while last_point is not None:
        last_point = came_from[last_point]
        if last_point is not None:
            path.append((last_point.x, last_point.y))
            #print(last_point)

    path.reverse()

    print(path)
    dir_dists = path_to_dists(path)
    print(dir_dists)
    car_direction = Direction.NORTH

    for dist, dir in dir_dists:
        
        dir_diff = dir - car_direction
        print(dir, car_direction, dir_diff)
        if dir_diff % 4 == 3:
            turn_90_left()
            car_direction = dir 
            print("turned left")
        elif dir_diff % 4 == 1:
            turn_90_right()
            car_direction = dir
            print("turned right")
        dist_to_time(dist)
        # break
        


if __name__ == '__main__':
    try:
        main()
    finally:
        fc.stop()
# [(6, N), (4, E), (7, )]