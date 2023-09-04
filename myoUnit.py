import multiprocessing
from pyomyo import Myo, emg_mode
from quaternion import Quaternion
from madgwickahrs import MadgwickAHRS
import numpy as np
import socket
import os
import time
import math
import math



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

host, port = "127.0.0.1", 25001
data = "1,2,3"

ahrs_filter = MadgwickAHRS(sampleperiod=1/50, beta=None, zeta=None)
roll, pitch, yaw=  str(5), str(6), str(7)
data = "{},{},{}".format(roll, pitch,yaw)
print (data)

start_time = time.time()

# SOCK_STREAM means TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# -------- Main Program Loop -----------

try:
	sock.connect((host, port))
	if __name__ == "__main__":
		p = multiprocessing.Process(target=worker, args=(q,))
		p.start()
		while 1:
				while not(q.empty()):
					imu = list(q.get())
					quat, acc, gyro = imu
					# print("Quaternions:", quat)
					# print("Acceleration:", acc)
					# print("Gyroscope:", gyro)
					quaternion_data = np.array(quat)  
					accelerometer_data = np.array(acc)  
					gyroscope_data = np.array(gyro)  
					quaternion_reescaled = quaternion_data/16384.0
					acc_rescaled = accelerometer_data/2048
					gyro_rescaled= gyroscope_data/16  
					magnitude = np.linalg.norm(acc_rescaled)
					acc_units = acc_rescaled / magnitude  # normalized
					gyro_units= gyro_rescaled*0.01745329251 
					ahrs_filter.update_imu(gyro_units, acc_units)
					estimated_orientation = ahrs_filter.quaternion
					euler_angles_radians = estimated_orientation.to_euler_angles()
					euler_angles_degrees = np.array(euler_angles_radians)* (180.0 / np.pi)
					# print("Euler Angles (Roll, Pitch, Yaw):", euler_angles_degrees)
					
					#Euler angles - madwick and mahony filter
					# roll = int(euler_angles_degrees[0])
					# pitch = int(euler_angles_degrees[1])
					# yaw = int(euler_angles_degrees[2])
					

					#Quaternions - from hardware 
					qw = quaternion_reescaled[0]
					qx = quaternion_reescaled[1]
					qy = quaternion_reescaled[2]
					qz = quaternion_reescaled[3]



					# data = "{},{},{}".format(qw, qx,qy,qz)
					def euler_from_quaternion(x, y, z, w):
						"""
						Convert a quaternion into euler angles (roll, pitch, yaw)
						roll is rotation around x in radians (counterclockwise)
						pitch is rotation around y in radians (counterclockwise)
						yaw is rotation around z in radians (counterclockwise)
						"""
						t0 = +2.0 * (w * x + y * z)
						t1 = +1.0 - 2.0 * (x * x + y * y)
						roll_x = math.atan2(t0, t1)
					
						t2 = +2.0 * (w * y - z * x)
						t2 = +1.0 if t2 > +1.0 else t2
						t2 = -1.0 if t2 < -1.0 else t2
						pitch_y = math.asin(t2)
					
						t3 = +2.0 * (w * z + x * y)
						t4 = +1.0 - 2.0 * (y * y + z * z)
						yaw_z = math.atan2(t3, t4)
					
						return roll_x, pitch_y, yaw_z # in radians

					
					roll, pitch, yaw = euler_from_quaternion( qx, qy, qz, qw)

					roll_deg = int(roll*(180/(math.pi)))
					pitch_deg = int(pitch*(180/(math.pi)))
					yaw_deg = int(yaw*(180/(math.pi)))
					print(f"Roll (degrees): {roll_deg}")
					print(f"Pitch (degrees): {pitch_deg}")
					print(f"Yaw (degrees): {yaw_deg}")
					data = "{},{},{}".format(roll_deg, yaw_deg,pitch_deg)
					# print(str(roll_deg)+" "+str(pitch)+" "+str(yaw))
					sock.sendall(data.encode("utf-8"))
					response = sock.recv(1024).decode("utf-8")
					print (response)
					# time.sleep(0.01)

finally:
	sock.close()