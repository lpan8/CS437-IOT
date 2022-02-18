#Base code is from obstacle_avoidance.py
import picar_4wd as fc
import sys
import tty
import termios
import asyncio
import time
import random
from picar_4wd import Ultrasonic

speed = 15

def main():
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue

        print(scan_list)
        # tmp = scan_list[3:7]
        tmp = scan_list[3:6]

        rand = random.randint(0,1)
        print(rand)

        print(tmp)
        us = fc.us
        dist = us.get_distance()
        print(dist)
        # if tmp != [2,2,2,2]:
        if tmp != [2,2,2]:
            if(dist <= 58):
                fc.stop()
                # time.sleep(1.5)
                time.sleep(1.0)
                fc.backward(10)
                time.sleep(1.0)
                # fc.stop()
                if(rand == 1):
                    fc.turn_right(speed)
                else:
                    fc.turn_left(speed)
        else:
            fc.forward(speed)

def detect_wall():   
    
    speed = 10
    fc.forward(10)
    us = fc.us
    while (1):
        dist = us.get_distance()
        print(dist)
        if dist <= 10:
            fc.stop()
            fc.backward(50)
            time.sleep(0.5)
            fc.stop()
            rand = random.randrange(500, 1000)
            for i in range(rand):
                if rand < 750:
                    fc.turn_right(100)
                else:
                    fc.turn_left(100)
            fc.forward(50)


if __name__ == "__main__":
    try: 
        main()
    #     detect_wall()
    finally: 
        fc.stop()

# def main():
#     try:
#         # move25()
#         detect_wall()
#     finally:
#         fc.stop()
# if __name__ == '__main__':
#     main()

