from DeviceRecognition import *
import picamera

# static flag to enable picamera code
USE_CAMERA = True
MOCK_IMG_PATH = "cvimages/image26.jpg"
#MOCK_IMG_PATH = "imgs/shuttlecock_lowres.jpg"
capture_path = "imgs/capture.jpg"

def processCommand(device_code = "none"):
	if device_code == "none":
		device_code = raw_input("Device Type: ")
		# Case on extracted device code!
  
	if (device_code == "V1"):
		device = ValveSmall()
	elif (device_code == "V2"):
		# device is large valve
		device = ValveLarge()
	elif (device_code == "V4"):
		# device is shuttlecock
		device = Shuttlecock()
	elif (device_code == "B"):
		# device is breaker box
		device = BreakerBox()
	else:
		return (0, 0, 0)
  
	if USE_CAMERA:
		camera.capture(self.capture_path,format = 'jpeg')
		path = self.capture_path
	else:
		path = MOCK_IMG_PATH

	retval = device.processImage(path)

	if not retval:
		print ("Detect FAILED!")
		#self.writeData(0,0,0)
		return
	else:
		print ("Successful detection!")
		(offset,orient,angle) = retval
		print(offset)
  		print(angle)
		#self.writeData(offset,orient,angle)
		return (offset, angle, orient)

print ("CV control running.")

#intialization, only set up this once once
camera.picamera.PiCamera()
#for example
device_code = "V2"

while 1:
	offset, angle, orient = processCommand(device_code)
 
print ("CV control stopped.")
