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

def main(): 
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

    stop = False
    count = 0
    
    while True:
        success, image = cap.read()
        if not success:
            sys.exit(
                'ERROR: Unable to read from webcam. Please verify your webcam settings.'
            )
        image = cv2.flip(image, 1)

        # Run object detection estimation using the model.
        detections = detector.detect(image)
        for detection in detections:
            category = detection.categories[0]
            class_name = category.label
            if class_name == 'stop sign':
                stop = True
                #print(class_name)

        scan_list = fc.scan_step(35)
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        #print(tmp)
        if tmp != [2,2,2,2]:
            fc.stop()
            fc.backward(50)
            time.sleep(0.5)
            #fc.stop()
            if 1 in tmp:
                fc.turn_right(speed)
            else:
                fc.turn_left(speed)
        else:
            if stop == True and count < 1:
                print("saw stop sign")
                fc.stop()
                time.sleep(5)
                stop = False
                fc.forward(speed)
                count += 1
            else:
                fc.forward(speed)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try: 
        main()
    finally:
        fc.stop()
