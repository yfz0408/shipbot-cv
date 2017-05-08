from __future__ import division
import cv2
import numpy as np

ORIENT_UP = "H"
ORIENT_SIDE = "V"

VALVE_CLOSED = 0
VALVE_OPEN = 1

DISTANCE_SCALE = -0.4375
ROBOTAXIS = 600

# Detects a shuttlecock device (front-facing only!)

class Shuttlecock:
    # HSV Color Range (Valve)
    blue_low = [100, 100, 100]
    blue_high = [135, 255, 255]

    # HSV Pipe Ranges
    white_low = [95, 0, 120]
    white_high = [110, 50, 240]

    # Valve Size Range
    area_min = 1000
    area_max = 10000
    
    orient = ORIENT_SIDE

    # Initialize numpy color arrays
    def __init__(self):
        self.thresh_low = np.array(self.blue_low, dtype="uint8")
        self.thresh_high = np.array(self.blue_high, dtype="uint8")
        self.pipe_low = np.array(self.white_low, dtype="uint8")
        self.pipe_high = np.array(self.white_high, dtype="uint8")

    def checkArea(self, area):
        return (area <= self.area_max and area >= self.area_min)

    def calculateAngle(self, ratio):
        if (self.orient == ORIENT_SIDE):
            if ratio > 2 :
                return 0
            else:
                return 1
        else:
            if ratio > 1:
                return 1
            else:
                return 0

    def getPipeAngle(self,hsv_image,image):
        pipe_area = 0
        
        maskroi = np.zeros((1200, 1600), np.uint8)
        myROI = [(900, 200), (900, 1000), (1600, 1000), (1600, 200)]
        cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)
        
        mask = cv2.inRange(hsv_image, self.pipe_low, self.pipe_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)
        img, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#        contours, hierarchy = cv2.findContours(
#            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if (area>2000):	
		print(area)
                x,y,w,h = cv2.boundingRect(cnt)
                pipe_area = pipe_area + area
        
        print(pipe_area)
        if pipe_area > 28000:
            self.orient = ORIENT_SIDE
            return True
        elif pipe_area > 18000:
            self.orient = ORIENT_UP
            return True
        else: 
            return False


    # Recieves a path to an image, returns ( offset, orientation, angle )
    def processImage(self, path):
        # ENSURE PATH IS VALID BEFORE USE!!!
        image = cv2.imread(path)
        height, width, channels = image.shape
        img_center = (width / 2, height / 2)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        maskroi = np.zeros((1200, 1600), np.uint8)
        myROI = [(900, 200), (900, 1000), (1600, 1000), (1600, 200)]
        cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)
        
        # mask it for the desired color as a binary
        mask = cv2.inRange(hsv_image, self.thresh_low, self.thresh_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # close any holes
        kernel = np.ones((5, 5), np.uint8)
        closed_thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # find the contours
        img, cnts, hierarchy = cv2.findContours(
            closed_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#        cnts, hierarchy = cv2.findContours(
#            closed_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if (not self.checkArea(area)):
                # not the right size for us
                continue
            x,y,w,h = cv2.boundingRect(cnt)
            x = x+w/2
            y = y+h/2
#            print(area)
#            area = dim[0] * dim[1]
            ratio = w/h
#            print(ratio)
            print(y)
            
            if (not self.getPipeAngle(hsv_image, image)):
                print("pipe not found")
                continue
            else:
                angle = self.calculateAngle(ratio)
                if self.orient == ORIENT_SIDE:
                    if angle == 1:
                        offset = ROBOTAXIS - (y - 130)
                    else:
                        offset = ROBOTAXIS - y
                else:
                    if angle == 0:
                        offset = ROBOTAXIS - (y - 130)
                    else:
                        offset = ROBOTAXIS - y
                        
                offset = offset*DISTANCE_SCALE
            return (int(offset), int(angle), self.orient)
        return False

class BreakerBox:
    # HSB Color Range (Valve)
    hsb_low = [0, 80, 140]
    hsb_high = [40, 255, 255]

    orient = ORIENT_SIDE
    theta = 0

    area_min = 800
    area_max = 2000
    ratio_max = 1

    def __init__(self):
        self.np_low = np.array(self.hsb_low, dtype="uint8")
        self.np_high = np.array(self.hsb_high, dtype="uint8")

    def inRange(self, area, ratio):
        if (area < self.area_min or area > self.area_max or ratio > self.ratio_max):
            return False
        else:
            return True

    def processImage(self, path):
        # load the image
        image = cv2.imread(path)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # masking non center area
        maskroi = np.zeros((1200, 1600), np.uint8)
        myROI = [(600, 200), (600, 1100), (1400, 1100), (1400, 200)]
        cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)

        # mask it for the desired color as a binary
        mask = cv2.inRange(hsv_image, self.np_low, self.np_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # find the contours
        img, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = cv2.contourArea(cnt)
            ratio = w / h
            ret = self.inRange(area, ratio)
            if (ret):
                #                    box = cv2.boxPoints(rect)
                #                    box = np.int0(box)
                    #x_offset = ROBOTAXIS - center[1]
                x = x+w/2
                y = y+h/2
                x_offset = ROBOTAXIS - y
                x_offset = x_offset * DISTANCE_SCALE
                print("Detected Breaker!")
                print(" - Horizontal offset: " + str(x_offset))
                # cv2.imshow("image", image)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                return (int(x_offset), int(self.theta), self.orient)
        return False


class ValveSmall:
    # HSV Color Range (Marker)
    mark_low = [30, 40, 120]
    mark_high = [70, 255, 255]

    # HSV Color Range (Valve)
    blue_low = [100, 100, 90]
    blue_high = [135, 255, 255]

    area_min = 2000
    area_max = 10000
#    orient = ORIENT_SIDE

    rf_min = 0.75
    rf_max = 1.3

    rp_min = .1
    rp_max = .75

    def __init__(self):
        self.np_low = np.array(self.blue_low, dtype="uint8")
        self.np_high = np.array(self.blue_high, dtype="uint8")

        self.mark_low = np.array(self.mark_low, dtype="uint8")
        self.mark_high = np.array(self.mark_high, dtype="uint8")

    def inRange(self, area, ratio):
        if (area < self.area_min):
            return (False, None)
    
        print("ratio was: " + str(ratio))
        if (ratio < .75):
            return (True, ORIENT_UP)
        else:
            return (True, ORIENT_SIDE)


    def findMarker(self, image, hsv_image):
        maskroi = np.zeros((1200, 1600), np.uint8)
        myROI = [(600, 200), (600, 1000), (1600, 1000), (1600, 200)]
        cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)

        mask = cv2.inRange(hsv_image, self.mark_low, self.mark_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # find the contours within bounds
        image, contours, hierarchy = cv2.findContours(
            output_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 50:
                rect = cv2.minAreaRect(cnt)
                center, dim, angle = rect
                return center
        return False

    def calculateAngle(self, center, point):
        x = point[0] - center[0]
        y = center[1] - point[1]
        rad = np.arctan2(y, x)
        deg = np.degrees(rad)
        deg = deg * -1
        if deg < 0:
            deg = deg + 360
        return deg

    def processImage(self, path):
        # load the image
        image = cv2.imread(path)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        maskroi = np.zeros((1200, 1600), np.uint8)
        myROI = [(700, 200), (700, 1000), (1600, 1000), (1600, 200)]
        cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)
        
        # mask it for the desired color as a binary
        mask = cv2.inRange(hsv_image, self.np_low, self.np_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # find the contours
        image, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            x = x+w/2
            y = y+h/2
            center = [x,y]
            if (w > 0 and h > 0):
                area = cv2.contourArea(cnt)
                ratio = w/h
                ret, orient = self.inRange(area, ratio)
                if (ret):
                    #print(area)
                    x_offset = ROBOTAXIS - center[1]
                    x_offset = x_offset * DISTANCE_SCALE
                    # cv2.drawContours(image,[box],0,(0,0,255),3)
                    mark_center = self.findMarker(image, hsv_image)
                    if not mark_center:
                        print("Didn't find mark!")
                        theta = 0
                    else:
                        theta = self.calculateAngle(center, mark_center)

                    print("Detected small valve!")
                    print(" - Horizontal offset: " + str(x_offset))
                    print(" - Measured Angle: " + str(theta))
                    print(" - Orient: " + orient)
                    # cv2.imshow("image", image)
                    # cv2.waitKey(0)
                    # cv2.destroyAllWindows()
                    return (int(x_offset), int(theta), orient)
        return False
# Detects a large valve (orange!)


class ValveLarge:
    device_id = 2

    # Target extracted HSB value
    hsb_target = [9, 194, 255]

    # HSB Color Range (Valve)
    hsb_low = [0, 80, 140]
    hsb_high = [80, 255, 255]

    # HSV Color Range (Marker)
    mark_low = [30, 40, 120]
    mark_high = [80, 180, 255]

    area_min = 4000

    rf_min = 0.5
    rf_max = 1.5

    rp_min = 1.5
    rp_max = 3.0

    def __init__(self):
        self.thresh_low = np.array(self.hsb_low, dtype="uint8")
        self.thresh_high = np.array(self.hsb_high, dtype="uint8")

        self.mark_low = np.array(self.mark_low, dtype="uint8")
        self.mark_high = np.array(self.mark_high, dtype="uint8")

    def inRange(self, area, ratio):
        if (area < self.area_min):
            #print (" * Bad Area: " + str(area))
            return (False, None)
        if ratio > 1.5:
            if (ratio > self.rp_max or ratio < self.rp_min):
                #print (" * Bad Ratio: " + str(ratio))
                return (False, None)
            else:
                return (True, ORIENT_SIDE)
        else:
            if (ratio > self.rf_max or ratio < self.rf_min):
                #print (" * Bad Ratio: " + str(ratio))
                return (False, None)
            else:
                return (True, ORIENT_SIDE)

    def inBounds(self, bounds, sel):
        (bound_x, bound_y) = bounds[0]
        (bound_w, bound_h) = bounds[1]
        bound_xmax = bound_x + bound_w
        bound_ymax = bound_y + bound_h

        (sel_x, sel_y) = sel[0]
        if (sel_x < bound_x or sel_x > bound_xmax):
            return False
        if (sel_y < bound_y or sel_y > bound_ymax):
            return False
        return True

    def findMarker(self, image, hsv_image, bounds):
        maskroi = np.zeros((1200, 1600), np.uint8)
        myROI = [(800, 200), (800, 1100), (1600, 1100), (1600, 200)]
        cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)

        mask = cv2.inRange(hsv_image, self.mark_low, self.mark_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # find the contours within bounds
        img, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if (area > 100):
		print(area)
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(image, [box], 0, (0, 255, 0), 3)
                center, dim, angle = rect
                return center
        return False

    def calculateAngle(self, center, point):
        x = point[0] - center[0]
        y = center[1] - point[1]
        rad = np.arctan2(y, x)
        deg = np.degrees(rad)
        deg = deg * -1
        if deg < 0:
            deg = deg + 360
        return deg

    def processImage(self, path):
        # Load image from path as HSV
        image = cv2.imread(path)
        height, width, channels = image.shape
        img_center = (width / 2, height / 2)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #maskroi = np.zeros((1200, 1600), np.uint8)
        #myROI = [(600, 200), (600, 1100), (1600, 1100), (1600, 200)]
        #cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        #hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)        
        
        # Mask out for desired color
        mask = cv2.inRange(hsv_image, self.thresh_low, self.thresh_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # close any holes
        kernel = np.ones((5, 5), np.uint8)
        closed_thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # find the contours
        img, contours, hierarchy = cv2.findContours(
            closed_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            center, dim, angle = rect
            if (dim[0] > 0 and dim[1] > 0):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                area = dim[0] * dim[1]
                ratio = dim[1] / dim[0]
                ret, orient = self.inRange(area, ratio)
                if (ret):
                    x_offset = ROBOTAXIS - center[1]
                    x_offset = x_offset * DISTANCE_SCALE
                    cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
                    mark_center = self.findMarker(image, hsv_image, rect)
                    if not mark_center:
                        print("Did not find marker")
                        return False
                    else:
                        theta = self.calculateAngle(center, mark_center)

                    print("Detected large valve!")
                    print(" - Horizontal offset: " + str(x_offset))
                    print(" - Angle: " + str(theta))
                    #cv2.imshow("image", image)
                    # cv2.waitKey(0)
                    # cv2.destroyAllWindows()
                    return (int(x_offset), int(theta), orient)

        #cv2.imshow("image", image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return False
