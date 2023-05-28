import multiprocessing
from pyomyo import Myo, emg_mode
import os
import socket
import time
import numpy as np
host, port = "127.0.0.1", 25001
data_ref = "0,0,0"
import math
# from ahrs.filters import Madgwick
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

# -------- Main Program Loop -----------
# SOCK_STREAM means TCP socket
# Connect to the server and send the data

    
if __name__ == "__main__":
	p = multiprocessing.Process(target=worker, args=(q,))
	p.start()
	start_time = time.time()
	# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	gx, gy, gz, ax, ay, az = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
	dt = 0.01  # time step in seconds
	
	try:
		# sock.connect((host, port))
		while True:
			while not(q.empty()):
				imu = list(q.get())
				quat, acc, gyro = imu
				# print("Quaternions:", quat)
				# print("Acceleration:", acc)
				# print("Gyroscope:", gyro)
				# data = str(acc)+","+str(gyro)
				gyro_str = str(gyro)
				acc_str = str(acc)
				quat_str=str(quat)

				gyro_str = gyro_str.strip('()') # remove parentheses
				gyro_str= gyro_str.split(', ')
				gx, gy, gz = float(gyro_str[0]), float(gyro_str[1]), float(gyro_str[2])

				acc_str = acc_str.strip('()') # remove parentheses
				acc_str= acc_str.split(', ')
				ax, ay, az = float(acc_str[0]), float(acc_str[1]), float(acc_str[2])

				quat_str = quat_str.strip('()') # remove parentheses
				quat_str= quat_str.split(', ')
				qx, qy, qz ,qw= float(quat_str[0]), float(quat_str[1]), float(quat_str[2]),float(quat_str[3])

				gyro_data = np.array([gx, gy, gz])
				accel_data = np.array([ax, ay, az])
				q_gyro = np.concatenate(([0], gyro_data))

				# Get the Euler angles from the filter
				# roll, pitch, yaw = filter.quaternion.to_euler(degrees=True)

				# print("Roll: {:.2f} degrees".format(roll))
				# print("Pitch: {:.2f} degrees".format(pitch))
				print("qx: " + str(qx))

	except KeyboardInterrupt:
		print("Quitting")
		quit()
	# finally:
		# sock.close()

		

