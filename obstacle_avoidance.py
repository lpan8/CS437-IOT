import picar_4wd as fc
import time
import random
import argparse
import sys
import time

import cv2
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions
import utils
from threading import Thread
import threading
from queue import Queue
import time

speed = 30
# stop_lock = threading.Lock()
# kill_lock = threading.Lock()

# kill = False


# class thread(Thread):
#     def __init__(self, name):
#         threading.Thread.__init__(self)
#         self.name = name
#         self.kill_received = False
 
#     def run(self):
        
#         while not self.kill_received:
#             # your code
#             print self.name, "is active"
#             time.sleep(1)

#def thread1(threadname):
def main(): 
    model = 'efficientdet_lite0.tflite'
    camera_id = 0
    num_threads = 4
    enable_edgetpu = False
    width = 640
    height = 480
    stop = False

    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()

        # Start capturing video input from the camera
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Initialize the object detection model
    options = ObjectDetectorOptions(
        num_threads=num_threads,
        score_threshold=0.3,
        max_results=3,
        enable_edgetpu=enable_edgetpu)
    detector = ObjectDetector(model_path=model, options=options)
    
    while True:
        success, image = cap.read()
        if not success:
            sys.exit(
                'ERROR: Unable to read from webcam. Please verify your webcam settings.'
            )

        counter += 1
        image = cv2.flip(image, 1)

        # Run object detection estimation using the model.
        detections = detector.detect(image)
        for detection in detections:
            category = detection.categories[0]
            class_name = category.label
            if class_name == 'stop sign':
                #stop_lock.acquire()
                stop = True
                #stop_lock.release()
                #print(class_name)
        # if kill == True:
        #     fc.stop()
        #     break
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        #print(tmp)
        if tmp != [2,2,2,2]:
            fc.turn_right(speed)
        else:
            if stop == True:
                print("saw stop sign")
                fc.stop()
                time.sleep(3)
                stop = False
                fc.forward(speed)
            else:
                fc.forward(speed)

        # scan_list = fc.scan_step(35)
        # if not scan_list:
        #     continue

        # tmp = scan_list[3:7]
        # #print(tmp)
        # if tmp != [2,2,2,2]:
        #     # fc.stop()
        #     # fc.backward(50)
        #     # time.sleep(0.5)
        #     # fc.stop()
        #     # rand = random.randrange(500, 1000)
        #     # for i in range(rand):
        #     #     if rand < 750:
        #     #         fc.turn_right(200)
        #     #     else:
        #     #         fc.turn_left(200)
        #     fc.turn_right(speed)
        # else:
        #     if stop == True:
        #         print("saw stop sign")
        #         fc.stop()
        #         time.sleep(3)
        #         stop = False
        #         fc.forward(speed)
        #     else:
        #         fc.forward(speed)

# def thread2(threadname):
#     global stops
#     model = 'efficientdet_lite0.tflite'
#     camera_id = 0
#     num_threads = 3
#     enable_edgetpu = False
#     width = 640
#     height = 480

#     # Variables to calculate FPS
#     counter, fps = 0, 0
#     start_time = time.time()

#     # Start capturing video input from the camera
#     cap = cv2.VideoCapture(camera_id)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

#     # Initialize the object detection model
#     options = ObjectDetectorOptions(
#         num_threads=num_threads,
#         score_threshold=0.3,
#         max_results=3,
#         enable_edgetpu=enable_edgetpu)
#     detector = ObjectDetector(model_path=model, options=options)

#      # Continuously capture images from the camera and run inference
#     while cap.isOpened():
#         success, image = cap.read()
#         if not success:
#             sys.exit(
#                 'ERROR: Unable to read from webcam. Please verify your webcam settings.'
#             )

#         counter += 1
#         image = cv2.flip(image, 1)

#         # Run object detection estimation using the model.
#         detections = detector.detect(image)
#         for detection in detections:
#             category = detection.categories[0]
#             class_name = category.label
#             if class_name == 'stop sign':
#                 stop_lock.acquire()
#                 stop = True
#                 stop_lock.release()
#             print(class_name)
#         if cv2.waitKey(1) == 27:
#             break
#     if kill == True:
#         cap.release()
#         cv2.destroyAllWindows()

# def has_live_threads(threads):
#     return True in [t.isAlive() for t in threads]

# def main():
#     fc.stop()
    # threads = []
    # thread_1 = Thread(target=thread1, args=("drive",))
    # thread_2 = Thread(target=thread2, args=("detect",))
    # thread_1.start()
    # thread_2.start()
    # threads.append(thread_1)
    # threads.append(thread_2)


    # while has_live_threads(threads):
    #     try:
    #         time.sleep(1)
    #         # synchronization timeout of threads kill
    #         #[t.join(1) for t in threads if t is not None and t.isAlive()]
    #     except KeyboardInterrupt:
    #         kill_lock.acquire()
    #         kill = True
    #         kill_lock.release()
            # thread1.join()
            # thread2.join()


if __name__ == "__main__":
    #main()
    try: 
        main()
    finally:
        fc.stop()
        
        
# fc.stop()
# thread1 = Thread(target=thread1, args=("drive",))
# thread2 = Thread(target=thread2, args=("detect",))

# try:
#     thread1.start()
#     thread2.start()
# finally:
#     fc.stop()
#     thread.join()
#     thread2.join()