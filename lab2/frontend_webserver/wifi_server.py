from cmath import e
from hashlib import sha256
import socket

import picar_4wd as fc
import sys
import tty
import termios
import asyncio
import picar_4wd.utils as util
import time

HOST = "192.168.1.180" # IP address of your Raspberry PI
PORT = 65432          # Port to listen on (non-privileged ports are > 1023)

power_val = 50
key = 'status'

def Keyborad_control(key):
    global power_val
    print("key[0],key[1],key",key[0],key[1],key)

    if key == b'87':
        fc.forward(power_val)
    elif key == b'65':
        fc.turn_right(power_val)
        time.sleep(.4)
        fc.forward(power_val)       
    elif key == b'83':
        fc.backward(power_val)
    elif key == b'68':
        fc.turn_left(power_val)
        time.sleep(.4)
        fc.forward(power_val)
    else:
        fc.stop()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    delimiter = '\n'
    try:
        while 1:
            client, clientInfo = s.accept()
            print("server recv from: ", clientInfo)
            data = client.recv(1024)      # receive 1024 Bytes of message in binary format
            send_data = ""

            if data != b"":
                print("data.decode",data.decode())
                print("data",data)
                s2 = data.decode()
                s1 = 'getData\r\n'
                print("s1,s2",len(s1),len(s2))

                if s1 != s2:
                    print ("Keyboard ")
                    Keyborad_control(data)
                
                cpu_usage = util.cpu_usage()
                send_data = str(cpu_usage) + delimiter

                cpu_temp = util.cpu_temperature()
                send_data += str(cpu_temp) + delimiter
                

                gpu_usage = util.gpu_temperature()
                send_data += str(gpu_usage) + delimiter

                disk_space = util.disk_space()
                send_data += str(disk_space) + delimiter
                
                ram_info = util.ram_info()
                send_data += str(ram_info) + delimiter
                
                pi_ip = util.getIP()
                send_data += str(pi_ip) + delimiter               
                
                power = util.power_read()
                send_data += str(power) + '\n\n'                    


                client.sendall(send_data.encode())                

                client.close()
    except Exception as e:
        print("Closing socket reason ",e)
        client.close()
        s.close()