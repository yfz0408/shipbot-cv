# OOP implementation of new CV controller
from DeviceRecognition import *
import picamera


class CVController:
    capture_path = "imgs/capture.jpg"  # path to write new captures to

    def __init__(self):
        self.camera = picamera.PiCamera()
        self.camera.resolution = (1600, 1200)

    # Command should be V1, V2, V3, or B
    def processCommand(self, device_code=None):
        # Case on extracted device code!
        if (device_code is "V1"):
            device = ValveSmall()
        elif (device_code is "V2"):
            # device is large valve
            device = ValveLarge()
        elif (device_code is "V3"):
            # device is shuttlecock
            device = Shuttlecock()
        elif (device_code is "B" or device_code is "A"):
            # device is breaker box
            device = BreakerBox()
        else:
            print("Unrecognized device code.")
            return (0, 0, "V")
        self.camera.resolution = (1600, 1200)
        self.camera.capture(self.capture_path, format='jpeg')
        retval = device.processImage(self.capture_path)
        if not retval:
            print("Detect FAILED!")
            return (0, 0, "V")
        else:
            print("Successful detection!")
        return retval
