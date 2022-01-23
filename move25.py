import RPi.GPIO as GPIO
import time, math
import threading
import picar_4wd as fc

speed = 30

class Speed():
    def __init__(self, pin):
        self.speed_counter = 0
        self.speed = 0
        self.last_time = 0
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.timer_flag = True
        self.timer = threading.Thread(target=self.fun_timer, name="Thread1")

    def start(self):
        self.timer.start()
        # print('speed start')

    def print_result(self, s):
        print("Rising: {}; Falling: {}; High Level: {}; Low Level: {}".format(s.count("01"), s.count("10"), s.count("1"), s.count("0")))

    def fun_timer(self):
        while self.timer_flag:
            l = ""
            for _ in range(100):
                l += str(GPIO.input(self.pin))
                time.sleep(0.001)
            # self.print_result(l)
            count = (l.count("01") + l.count("10")) / 2
            rps = count / 20.0 * 10
            self.speed = round(2 * math.pi * 3.3 * rps, 2)

    def __call__(self):
        return self.speed

    def deinit(self):
        self.timer_flag = False
        self.timer.join()


def test3():
    speed4 = Speed(25)
    speed4.start()
    # time.sleep(2)
    fc.backward(100)
    x = 0
    for i in range(20):
        time.sleep(0.1)
        speed = speed4()
        x += speed * 0.1
        print("%smm/s"%speed)
    print("%smm"%x)
    speed4.deinit()
    fc.stop()


if __name__ == "__main__":
    test3()
    # move25()
        