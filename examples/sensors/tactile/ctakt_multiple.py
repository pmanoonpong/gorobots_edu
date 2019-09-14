# API imports #
from timeit import default_timer as timer
import datetime
import serial
import numpy as np
import time
import cv2
import threading
import socket
import sys
import fcntl, os
import csv
from Queue import Queue
#from thread import allocate_lock


show_wifi_1 = True
save_to_file = False
#ip = '10.125.18.169'
ip = '192.168.1.100'
img = np.zeros((700, 1000, 3), dtype='uint8')
#img = cv2.imread('.jpg', 0)

# this sets the background color (R,G,B)
img[:,:,0] = 160
img[:,:,1] = 150
img[:,:,2] = 140
cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
cv2.imshow('image', img)
import matplotlib.pyplot as plt
#plt.ion()
#plt.imshow(img)
#cv2.waitKey(20)
#cv2.resizeWindow('image', 911,911)

########################################################






class Sensor:
	global time_start_saving
	def __init__(self, name, ip, port, offset_x, offset_y, flip_long, flip_short, logging):	
		self.name = name
		self.ip = ip
		self.port = port
		self.offset_x = offset_x
		self.offset_y = offset_y
		self.flip_long = flip_long
		self.flip_short = flip_short
		self.logging = logging
		self.queue = Queue();
		self.data_queue = Queue();
		self.frameReadingError = False
		self.synchronized = False
		self.sensitivity = 0 # 0 to 4, 0 is the most sensitive
		self.data_available = False
		self.values_2d = np.zeros( (10,25), dtype=np.uint8 ) #values 2d array, rows, cols
		self.lock = threading.Lock()
		self.time_fps = 0
		self.fps = 0
		self.counter_fps = 0
		self.value = 100
	

		##################################################################
		#####-this holds info for the thumb finger (1)
		##################################################################

		self.offset2 = np.empty([11], dtype=np.uint16)
		self.offset1 = np.empty([11], dtype=np.uint16)

		self.offset_y = self.offset_y # 420
		self.stepj = np.empty([25],dtype = float)
		self.offset1 = self.offset1[::1]
		self.offset1 = np.array(self.offset1) * 3
		self.offset2 = self.offset2[::1]
		self.offset2 = np.array(self.offset2) * 3.1
		for i in range(0, 11):
			self.offset1[i] = self.offset_x - 0 # 400 - i
			self.offset2[i] = self.offset_x - 300 - 0 # 100 - i to get a tilt
			self.stepj[i] = (self.offset1[i] - self.offset2[i])/25.0
		self.stepi = 12
		#get all the coordinates of the polygons
		self.p1x = np.empty([250],dtype = np.int32)
		self.p2x = np.empty([250],dtype = np.int32)
		self.p3x = np.empty([250],dtype = np.int32)
		self.p4x = np.empty([250],dtype = np.int32)
		self.pts = np.empty([10,25, 4, 2],dtype = np.int32 )
		for i in range(0, 10):
			for j in range (0, 25):
				self.p1x[i*25+j] = self.offset2[i] + (self.stepj[i] * j)
				self.p2x[i*25+j] = self.p1x[i*25+j] + self.stepj[i] 
				self.p4x[i*25+j] = self.offset2[i+1] + (self.stepj[i+1]*j)
				self.p3x[i*25+j] = self.p4x[i*25+j] + (self.stepj[i])
				self.pts[i][j] = np.array([[self.p1x[i*25+j], self.offset_y + i*self.stepi], [self.p2x[i*25+j], self.offset_y + i*self.stepi], [self.p3x[i*25+j], self.offset_y + (i+1)*self.stepi], [self.p4x[i*25+j], self.offset_y + (i+1)*self.stepi]], np.int32)
				#self.pts[i][j] = self.pts[i][j].reshape((-1,1,2))
				self.pts[i][j] = self.pts[i][j].reshape((-1,2))
	def sensor_info(self):
		print "Name: %s " % self.name
		print "IP: %s " % self.ip
		print "Port: %s " % self.port 
		print "Offset X: %s " % self.offset_x
		print "Offset Y: %s " % self.offset_y
		print "Flip long axis: %s " % self.flip_long
		print "Flip short axis: %s " % self.flip_short
		print "Logging: %s " % self.logging
	# Function definition to initialize the thread for reading from the current sensor on the specified port
	def sensor_init(self):
		###-this is related to the UDP server running for receiving messages from the sensor
		# Create a UDP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		#self.sock.settimeout(1)
		#fcntl.fcntl(self.sock, fcntl.F_SETFL, os.O_NONBLOCK)
		# Bind the socket to the port
		self.server_address = (self.ip, self.port)
		#server_address = ('localhost', 10000)
		print >>sys.stderr, 'starting up on %s port %s' % self.server_address
		self.sock.bind(self.server_address)
		self.sock.setblocking(0)
		self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,900)
		time_start_saving = timer()
		self.thread_one = ReadFromSensor(self)
		self.thread_one.setDaemon(True)
		self.thread_one.start()
	# Function definition for scanning one frame over wifi
	def scan_frame_wifi(self):
		global time_start_saving
		"This function will read one tactile image from wifi"
		#print >>sys.stderr, '\nwaiting to receive message'
		#result = 1
		if self.synchronized == False:
			try:
				self.data, self.address = self.sock.recvfrom(504)
			except socket.error:
				pass
			else:
			
				print >>sys.stderr, 'data received'
		  		self.results = bytearray(self.data)
			
				for i in range(0, len(self.data)):
					self.data_queue.put(self.results[i])
				for i in range(0, len(self.data)):
					a = self.data_queue.get()
					if self.logging == True:
						print >>sys.stderr, 'element' 
		 			if a == 255:
						self.synchronized = True
						if self.logging == True:
							print >>sys.stderr, 'Synchronized.' 
						break
					else:
						if self.logging == True:
							print >>sys.stderr, 'Not yet syncrhonized. %d element %d' %(a, i) 

		#print >>sys.stderr, 'received %s bytes from %s' % (len(data), address)
		try:
			self.data, self.address = self.sock.recvfrom(504)
		except socket.error:
			pass
		else:
			if self.logging == True:
				print >>sys.stderr, 'received %s bytes from %s' % (len(self.data), self.address)
			#print >>sys.stderr, data
			self.frameReadingError = False
			self.time_receive_start = timer()
			self.results_1 = bytearray(self.data)
			if (len(self.data)!=504):
				for i in range(0, 503):
					self.data_queue.put(0)
				self.data_queue.put(255)
			else:
				for i in range(0, 504):
					self.data_queue.put(self.results_1[i])

			self.checksum = 0
			for i in range (0, 502):
				self.checksum ^= self.results_1[i]
			#a = data_queue.get()
			#while a != 255:
			#	a = data_queue.get()
			if self.logging == True:
				print >>sys.stderr, '%d length %d' %(0, self.data_queue.qsize())
			#results_1 = bytearray(502)
			self.values_16bit = 253 * [0]
			self.curvature = 0
			for i in range (0, 251):
				if (self.data_queue.qsize()>0):
					self.values_16bit[i]= self.data_queue.get()+ self.data_queue.get()*256 # this will convert from 500 values 8 bit array to 250 values, 16 bit array
					#print i
			self.values_16bit[251]= self.data_queue.get() # the hash
			self.values_16bit[252]= self.data_queue.get() # the end frame
			if self.values_16bit[252] != 255:
				#print results_1
				a = 0
			if (len(self.values_16bit) < 253):
				self.frameReadingError = True
				if self.logging == True:
					print("framing error")
			else:
				if (self.checksum != self.values_16bit[251]):
				#if (checksum < 0): #cannot happen
					if self.logging == True:
						print >>sys.stderr, 'checksum error %d vs %d' % (self.checksum, self.values_16bit[251])
					#synchronized = False
				else:
					self.counter_fps+=1


					#self.lock.acquire()
					for i in range(0, 10):
						for j in range (0, 25):
						#values_1[i] =  int(30*np.log(0.0004*sensitivity*(values_16bit[i]+1))) #
							#self.values_1[i*25+j] =  self.values_16bit[i*25+j] >> self.sensitivity
							self.values_2d[i][j] = min(254, self.values_16bit[i*25+j] >> self.sensitivity)
					self.curvature = self.values_16bit[250] >> self.sensitivity#holding curvature value 
					if (self.flip_long == True):
						self.values_2d = np.fliplr(self.values_2d)
					if (self.flip_short == True):
						self.values_2d = np.flipud(self.values_2d)



						#print >>sys.stderr, '%d ' %values_1[i]
					#values_1 = list(results_1) * sensitivity
					#print(values_1[i])

					self.data_available = True
					if self.counter_fps == 10:
						self.counter_fps = 0
						self.timef_fps = timer()
						self.time_fps = self.timef_fps - self.time_fps #current time of the 10th frame - first time of the first frame
						self.fps = 10/self.time_fps
						self.time_fps = self.timef_fps
						self.counter_fps = 0
					#self.lock.release()

			self.time_receive_stop= timer()
			self.time_for_saving = self.time_receive_stop - time_start_saving
			self.delta= self.time_receive_stop - self.time_receive_start
	# Function definition for showing GUI
	def show_sensor_gui(self):
		global img

		# for i in range(0, 10):
	# 		for j in range (0, 25):
	# 			value = int(values_1[i*25+j])
	# 			pts_1 = np.array([[p1x_1[i*25+j], offset_y_1 + i*stepi_1], [p2x_1[i*25+j], offset_y_1 + i*stepi_1], [p3x_1[i*25+j], offset_y_1 + (i+1)*stepi_1], [p4x_1[i*25+j], offset_y_1 + (i+1)*stepi_1]], np.int32)
	# 			pts_1 = pts_1.reshape((-1,1,2))
	# 			cv2.fillConvexPoly(img,pts_1,(value,value,value), lineType=4, shift=0)
	# 			cv2.polylines(img,[pts_1],True,(100,100,100))

			
		for i in range(0, 10):
			for j in range (0, 25):
				self.value = int(self.values_2d[i][j])
				#print(" " + repr(value))
				cv2.fillConvexPoly(img,self.pts[i][j],(self.value,self.value,self.value), lineType=4, shift=0)
				cv2.polylines(img,[self.pts[i][j]],True,(100,100,100))
	
		#resized_img = cv2.resize(img,None,fx=1, fy=1, interpolation=cv2.INTER_NEAREST)
		#cv2.imshow('image', resized_img)
	
		#return;

# configure the serial connections (the parameters differs on the device you are connecting to)
# Define a function for the thread
class ReadFromSensor(threading.Thread):
	def __init__(self, current_sensor):
		threading.Thread.__init__(self)
		self.current_sensor = current_sensor
	
	def run(self):
	
		while 1:
			self.current_sensor.scan_frame_wifi()
			
			#for i in range(0, 10):
			#	for j in range (0, 25):
				#values_1[i] =  int(30*np.log(0.0004*sensitivity*(values_16bit[i]+1))) # 
					#self.values_1[i*25+j] =  self.values_16bit[i*25+j] >> self.sensitivity
					#self.current_sensor.values_2d[i][j] = (i*3+j) %254
			#self.current_sensor.data_available = True
			#self.current_sensor.frameReadingError = False
			time.sleep(0.0010)

class ShowGUI(threading.Thread):
	global show_wifi_1
	global save_to_file
	
	def __init__(self, *args):
		threading.Thread.__init__(self)
		self.sensors = args
		self.eventStop = threading.Event()
		for i in range(0,6):
			with open('log%d.csv'%i, 'wb') as f:
				self.writer = csv.writer(f)
				f.close()
		
	def run(self):
		status = 0
		#time.sleep(1)
		while not self.eventStop.is_set():
			#self.sensors[0].sensor_info()
			#time.sleep(1.11)
			#print len(self.sensors)
			time_gui_start= timer()
			img_with_text = np.copy(img)
			font = cv2.FONT_HERSHEY_COMPLEX
			#status = status + 1
			for i in range(0, len(self.sensors)): #shows the graphical part for each sensor
				if (self.sensors[i].frameReadingError == False):		
					#self.sensors[i].lock.acquire()
					#self.sensors[i].lock.release()		
					if (True): 
							#print "Data available %s" %self.sensors[i].data_available
							if (self.sensors[i].data_available == True):			
								self.sensors[i].show_sensor_gui()
								if (save_to_file == True):
									with open(r'log%d.csv'%i, 'a') as f:
										#print "Data available"
										data_to_save = np.empty([253],dtype = np.int32)  #create empty array
										data_to_save[0] = self.sensors[i].time_for_saving * 1000 #save the timestamp
										data_to_save[1] = self.sensors[i].port #port
										data_to_save[2:253] = self.sensors[i].values_16bit[0:251]
										self.writer = csv.writer(f)
										self.writer.writerow(data_to_save)
								
								cv2.putText(img_with_text, "{0:3d}".format(self.sensors[i].curvature) + " curvature %s"%self.sensors[i].name,(self.sensors[i].offset_x-330,self.sensors[i].offset_y+150), font, .7,(0,0,0))	
								cv2.putText(img_with_text, "{0:3.3f}".format(self.sensors[i].fps) + " fps %s"%self.sensors[i].name,(self.sensors[i].offset_x-330,self.sensors[i].offset_y+170), font, .7,(0,0,0))			
					#resized_img = cv2.resize(img,None,fx=1, fy=1, interpolation=cv2.INTER_NEAREST)
			time_gui_end = timer()
			time_delta_gui = time_gui_end- time_gui_start
			cv2.putText(img_with_text, "{0:.3f}".format(time_delta_gui) + " s GUI refresh",(500,20), font, .7,(0,0,0))		
			cv2.imshow('image', img_with_text)
			#cv2.waitKey(5)
			#time.sleep(0.010)
			


read = 0
#### Sensor ( NAME, PORT, OFFSET_X, OFFSET_Y, FLIP_LONG_AXES, FLIP_SHORT_AXES, logging_enabled)
s1 = Sensor('Sensor 1', ip, 10001, 400, 50, False, True, False)
s1.sensor_init()
###########s1.sensor_info()
s2 = Sensor('Sensor 2', ip, 10002, 400, 300, True, True, False)
s2.sensor_init()

s3 = Sensor('Sensor 3', ip, 10003, 400, 500, True, True, False)
s3.sensor_init()

s4 = Sensor('Sensor 4', ip, 10004, 800, 50, True, True, True)
s4.sensor_init()

s5 = Sensor('Sensor 5', ip, 10005, 800, 250, True, True, False)
s5.sensor_init()

s6 = Sensor('Sensor 6', ip, 10006, 900, 400, True, True, False)
s6.sensor_init()


time_start_saving = timer()

while 1 :
	# get keyboard input
	
	read = raw_input(">> ")
        # Python 3 users
        # input = input(">> ")
	# if read == "exit":
# 		try:
# 			t1.eventStop.set()
# 			cv2.destroyAllWindows()
# 		except:
# 			print "Error: unable to exit"
# 		exit()
	if read == "l": #this starts logging
		if save_to_file == True:
			save_to_file = False
			print "Saving to file ended"
		else:
			save_to_file = True
			time_start_saving = timer()
			print "Saving to file started"
	if read == "v": #starts the visualizer
		try:
			#t1 = ReadFromSensors()
			t2 = ShowGUI(s1, s2, s3, s4, s5, s6) # the list should contain all the sensors initialized
			#t1.setDaemon(True)
			t2.setDaemon(True)
			#t1.start()
			t2.start()
		except:
			print "Error: unable to start thread"


		
	