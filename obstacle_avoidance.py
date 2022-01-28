import picar_4wd as fc
import time
import random

speed = 30

def main():
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        print(tmp)
        if tmp != [2,2,2,2]:
            fc.stop()
            fc.backward(50)
            time.sleep(0.5)
            fc.stop()
            rand = random.randrange(500, 1000)
            for i in range(rand):
                if rand < 750:
                    fc.turn_right(200)
                else:
                    fc.turn_left(200)
        else:
            fc.forward(speed)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
