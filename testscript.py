from DeviceRecognition import ValveSmall, Shuttlecock

path = "imgs/V3/image45.jpg"

v = Shuttlecock()
ret = v.processImage(path)
print(ret)
