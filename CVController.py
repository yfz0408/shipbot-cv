from DeviceRecognition import *
import picamera

# static flag to enable picamera code
USE_CAMERA = False
MOCK_IMG_PATH = "cvimages/image26.jpg"
#MOCK_IMG_PATH = "imgs/shuttlecock_lowres.jpg"

class CVController:

	data_path = "CV.txt"
	capture_path = "imgs/capture.jpg"

	format_str = "@ 0\nOFFSET {offset}\nORIENT {orient}\nANGLE {angle}\n"

	killed = False

	def __init__(self):
		print ("Awaiting sync...")
		synced = False
		while not synced:
			file = open(self.data_path, 'r')
			for line in file:
				if "NO DATA" in line:
					synced = True
			file.close()
		print ("Sync acquired.")
		self.writeData(0, 0, 0)
		camera = picamera.PiCamera()

	def isActive(self):
		return not self.killed

	def processCommand(self):
		print ("Awaiting command...")
		cmd_recieved = False
		device_code = 0
		while not cmd_recieved:
			file = open(self.data_path, 'r')
			for line in file:
				if "@ 1" in line:
					cmd_recieved = True
				if cmd_recieved:
					if "STOP" in line:
						self.killed = True
					else:
						(key, value) = line.split(' ')
						if "DEVICE" in key:
							device_code = int(value.strip('\n'))
			file.close()

		if self.killed:
			print ("Recieved stop.")
			return
			
		# Case on extracted device code!
		if (device_code == 1):
			# device type: valve small
			device = ValveSmall()
		elif (device_code == 2):
			# device is large valve
			device = ValveLarge()
		elif (device_code == 3):
			# device is shuttlecock
			device = Shuttlecock()
		elif (device_code == 4):
			# device is breaker box
			device = BreakerBox()
		else:
			# device code is unrecognized or 0
			return

		if USE_CAMERA:
			self.capture()
			path = self.capture_path
		else:
			path = MOCK_IMG_PATH

		retval = device.processImage(path)

		if not retval:
			print ("Detect FAILED!")
			self.writeData(0,0,0)
			return
		else:
			print ("Successful detection!")
			(offset,orient,angle) = retval
			self.writeData(offset,orient,angle)
			return

	def capture(self):
		if not USE_CAMERA:
			return
		else:
			# TODO: add picamera capture here!
			self.camera.capture(self.capture_path,format = 'jpeg')                                   
			pass

	def writeData(self, offset, orientation, angle):
		msg = self.format_str.format(offset=offset, orient=orientation, angle=angle)
		file = open(self.data_path, 'w')
		file.write(msg)
		file.close()

print ("CV control running.")
c = CVController()
while c.isActive():
	c.processCommand()
print ("CV control stopped.")
