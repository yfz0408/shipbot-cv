from DeviceRecognition import ValveSmall, ValveLarge, Shuttlecock

path = "imgs/V3/V3_up_0_1.jpg"

v = Shuttlecock()
#v = ValveLarge()
ret = v.processImage(path)
print(ret)
