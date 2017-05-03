import cv2
import numpy as np

ORIENT_UP = "H"
ORIENT_SIDE = "V"

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

    thresh_wide_low = 1.5
    thresh_wide_high = 5.5

    thresh_narrow_low = .15
    thresh_narrow_high = .5

    # Valve Size Range
    area_min = 2000
    area_max = 7000

    # Pipe size range
    area_min_pipe = 6000
    area_max_pipe = 30000
    angle_thresh_pipe = 20

    # Initialize numpy color arrays
    def __init__(self):
        self.thresh_low = np.array(self.blue_low, dtype="uint8")
        self.thresh_high = np.array(self.blue_high, dtype="uint8")
        self.pipe_low = np.array(self.white_low, dtype="uint8")
        self.pipe_high = np.array(self.white_high, dtype="uint8")

    def checkArea(self, area):
        return (area <= self.area_max and area >= self.area_min)

    def checkRatio(self, ratio):
        if (abs(5 - ratio) < 2):
            # print(ratio)
            # print("hi")
            return (True, ORIENT_SIDE, 90)
        elif (abs(2 - ratio) < 1):
            # print(ratio)
            return (True, ORIENT_UP, 90)
        elif (abs(.5 - ratio) < .3):
            # print(ratio)
            return (False, None, None)
        # print(ratio)
        return False

    def getPipeAngle(self, hsv_image, image):
        mask = cv2.inRange(hsv_image, self.pipe_low, self.pipe_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)
        img, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            center, dim, angle = rect
            if (dim[0] > 0 and dim[1] > 0):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                area = dim[0] * dim[1]
                ratio = dim[1] / dim[0]
                cv2.drawContours(image, [box], 0, (255, 255, 0), 1)
                if (area > self.area_min_pipe and area < self.area_max_pipe):
                    cv2.drawContours(image, [box], 0, (255, 255, 0), 2)
                    if ((abs(ratio - 2) < .5) or (abs(ratio - 1) < .5)):
                        if (abs(angle) < 10):
                            cv2.drawContours(image, [box], 0, (0, 255, 0), 3)
                            #print("angle: " + str(angle))
                            #print("ratio: " + str(ratio))
                            #print("Found horizontal pipe?")
                            return 0
                        elif (abs(angle) > 80):
                            cv2.drawContours(image, [box], 0, (0, 255, 0), 3)
                            #print("angle: " + str(angle))
                            #print("ratio: " + str(ratio))
                            #print("Found horizontal pipe?")
                            return 0
                        else:
                            cv2.drawContours(image, [box], 0, (255, 0, 0), 3)
                            #print("angle: " + str(angle))
                            #print("ratio: " + str(ratio))
                            #print("found a vertical pipe maybe????")
                            return 90
        # print("hi")
        return 90

    # Recieves a path to an image, returns ( offset, orientation, angle )
    def processImage(self, path):
        # ENSURE PATH IS VALID BEFORE USE!!!
        image = cv2.imread(path)
        height, width, channels = image.shape
        img_center = (width / 2, height / 2)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

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

        for cnt in cnts:
            rect = cv2.minAreaRect(cnt)
            center, dim, angle = rect
            if (dim[0] <= 10 or dim[1] <= 10):
                # WAY too small, skip it!
                continue

            # Box a contour that matches our colors
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            area = dim[0] * dim[1]
            if (abs(angle) < 45):
                ratio = dim[1] / dim[0]
            else:
                ratio = dim[0] / dim[1]
            cv2.drawContours(image, [box], 0, (0, 0, 255), 1)

            if (not self.checkArea(area)):
                # not the right size for us
                continue

            cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
            # print(angle)
            ratio_check = self.checkRatio(ratio)
            if (ratio_check == False):
                # Move on, not of a known ratio
                continue

            cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
            ret, orient, meas_angle = ratio_check
            if (ret):
                # print("skipped pipe angle calc")
                # We could distinguish by ratio
                if (angle == 90):
                    offset = ROBOTAXIS - center[1]
                else:
                    offset = ROBOTAXIS - (center[1] - 100)
                # self.renderImage(image)
                return (int(offset * DISTANCE_SCALE), meas_angle, orient)
            else:
                # we couldn't distinguish
                #print("Device box angle: " + str(angle))
                pipe_angle = self.getPipeAngle(hsv_image, image)
                #print("Pipe angle: " + str(pipe_angle))
                # self.renderImage(image)
                if (abs(pipe_angle) - abs(angle)) < 20:
                    # We could distinguish by ratio
                    if (abs(angle) > 45):
                        offset = ROBOTAXIS - center[1]
                        return (int(offset * DISTANCE_SCALE), 0, ORIENT_SIDE)
                    else:
                        offset = ROBOTAXIS - (center[0] - 100)
                        return (int(offset * DISTANCE_SCALE), 90, ORIENT_SIDE)
                else:
                    if (abs(angle) >= 45):
                        offset = ROBOTAXIS - center[1]
                        return (int(offset * DISTANCE_SCALE), 0, ORIENT_SIDE)
                    else:
                        offset = ROBOTAXIS - (center[1] - 100)
                        return (int(offset * DISTANCE_SCALE), 0, ORIENT_SIDE)
        # self.renderImage(image)
        return (0, 0, ORIENT_SIDE)

    def renderImage(self, image):
        cv2.imshow("image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


class BreakerBox:
    # HSB Color Range (Valve)
    hsb_low = [0, 80, 120]
    hsb_high = [40, 255, 255]

    orient = ORIENT_SIDE
    theta = 0

    area_min = 1000
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
    mark_high = [70, 100, 255]

    # HSV Color Range (Valve)
    blue_low = [100, 100, 100]
    blue_high = [135, 255, 255]

    area_min = 2000
#    orient = ORIENT_SIDE

    rf_min = 0.75
    rf_max = 2

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
	
	ratio = 1/ratio
        print("ratio was: " + str(ratio))
        if ratio < .75:
            if (ratio > self.rp_max or ratio < self.rp_min):
                return (False, None)
            else:
                return (True, ORIENT_UP)
        else:
            if (ratio > self.rf_max or ratio < self.rf_min):
                return (False, None)
            else:
                return (True, ORIENT_SIDE)

    def inBounds(self, bounds, sel):
        (bound_x, bound_y) = bounds[0]
        (sel_x, sel_y) = sel[0]

        x_diff = abs(bound_x - sel_x)
        y_diff = abs(bound_y - sel_y)
        if (x_diff > 100):
            #print ("X out of bounds")
            return False
        if (y_diff > 100):
            #print ("Y out of bounds")
            return False
        return True

    def findMarker(self, image, hsv_image, bounds):
        maskroi = np.zeros((1200, 1600), np.uint8)
        myROI = [(600, 200), (600, 1000), (1600, 1000), (1600, 200)]
        cv2.fillPoly(maskroi, [np.array(myROI)], 255)
        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=maskroi)

        mask = cv2.inRange(hsv_image, self.mark_low, self.mark_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # find the contours within bounds
        img, contours, hierarchy = cv2.findContours(
            output_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 50:
                rect = cv2.minAreaRect(cnt)
                center, dim, angle = rect
    #            box = cv2.boxPoints(rect)
    #            box = np.int0(box)
                # cv2.drawContours(image,[box],0,(0,255,0),1)
                if self.inBounds(bounds, rect):
                    # cv2.drawContours(image,[box],0,(0,255,0),4)
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

        # mask it for the desired color as a binary
        mask = cv2.inRange(hsv_image, self.np_low, self.np_high)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        output_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(output_gray, 15, 255, cv2.THRESH_BINARY)

        # find the contours
        img, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            center, dim, angle = rect
#            center = (corner[0] + (dim[0]/2), corner[1] + dim[1]/2)

            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # cv2.drawContours(image,[box],0,(255,0,0),1)
            if (dim[0] > 0 and dim[1] > 0):
                area = dim[0] * dim[1]
                ratio = dim[1] / dim[0]
                ret, orient = self.inRange(area, ratio)
                if (ret):

                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    x_offset = ROBOTAXIS - center[1]
                    x_offset = x_offset * DISTANCE_SCALE
                    # cv2.drawContours(image,[box],0,(0,0,255),3)
                    mark_center = self.findMarker(image, hsv_image, rect)
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
                return (True, ORIENT_UP)

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
        myROI = [(600, 200), (600, 1100), (1600, 1100), (1600, 200)]
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
            if (area > 20):
                rect = cv2.minAreaRect(cnt)
                center, dim, angle = rect
                return center
        return (-1, -1)

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
                    # cv2.drawContours(image,[box],0,(0,0,255),3)
                    mark_center = self.findMarker(image, hsv_image, rect)
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
