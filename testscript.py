from DeviceRecognition import ValveSmall, Shuttlecock

path = "imgs/V3/V3_side_90_1.jpg"

v = Shuttlecock()
ret = v.processImage(path)
print(ret)
