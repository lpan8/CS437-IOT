import picar_4wd as fc
import sys
import tty
import termios
import asyncio
import time
import random

print("testing...")

def move25():
    speed = fc.Speed(25)
    speed.start()
    fc.forward(100)
    #time.sleep(0.5)
    x = 0
    for i in range(5):
        time.sleep(0.1)
        cur_speed = speed()
        x += cur_speed * 0.1
        print("%smm/s"%cur_speed)
    print("%smm"%x)
    speed.deinit()
    fc.stop()

def detect_wall():   
    
    fc.forward(50)
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

def main():
    try:
        move25()
        #detect_wall()
    finally:
        fc.stop()

if __name__ == '__main__':
    main()

