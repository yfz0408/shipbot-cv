from DeviceRecognition import ValveSmall, ValveLarge, Shuttlecock

path = "imgs/V2/V2_ang_5.jpg"

#v = Shuttlecock()
v = ValveLarge()
ret = v.processImage(path)
print(ret)
