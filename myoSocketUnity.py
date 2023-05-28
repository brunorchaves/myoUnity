import socket
import time
import multiprocessing
from pyomyo import Myo, emg_mode
import os
import socket
import numpy as np
import math

host, port = "127.0.0.1", 25001
x, y, z = 1.0, 2.0, 3.0

# SOCK_STREAM means TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def cls():
	# Clear the screen in a cross platform way
	# https://stackoverflow.com/questions/517970/how-to-clear-the-interpreter-console
    os.system('cls' if os.name=='nt' else 'clear')

# ------------ Myo Setup ---------------
q = multiprocessing.Queue()

def worker(q):
	m = Myo(mode=emg_mode.FILTERED)
	m.connect()
	
	def add_to_queue(quat, acc, gyro):
		imu_data = [quat, acc, gyro]
		q.put(imu_data)

	m.add_imu_handler(add_to_queue)
	
	# Orange logo and bar LEDs
	m.set_leds([255, 0, 255], [255, 0, 255])
	# Vibrate to know we connected okay
	m.vibrate(1)
	
	"""worker function"""
	while True:
		m.run()
	print("Worker Stopped")
        

try:
    # Connect to the server and send the data in a loop
    sock.connect((host, port))
    # -------- Main Program Loop -----------
    if __name__ == "__main__":
        p = multiprocessing.Process(target=worker, args=(q,))
        p.start()

        try:
            while True:
                while not(q.empty()):
                    
                    imu = list(q.get())
                    quat, acc, gyro = imu
                    quat_str=str(quat)

                    quat_str = quat_str.strip('()') # remove parentheses
                    quat_str= quat_str.split(', ')
                    qx, qy, qz ,qw= float(quat_str[0]), float(quat_str[1]), float(quat_str[2]),float(quat_str[3])

                    data = "{},{},{}".format(qx, qy, qz)
                    sock.sendall(data.encode("utf-8"))
                    response = sock.recv(1024).decode("utf-8")
                    print(response)
                    time.sleep(0.1)
                    # x += 0.05
                    # y += 0.05
                    # z += 0.05
                    print(qx)
        except KeyboardInterrupt:
            print("Quitting")
            quit()

except KeyboardInterrupt:
    print("Program terminated by user")

finally:
    sock.close()