import cv2
import numpy as np
import math

# Helper classes
class Point:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)
    
    def __mul__(self, f):
        return Point(self.x * f, self.y * f)

class FieldModel:
    def __init__(self):
        self.a = 12.0  # Field length
        self.b = 8.0   # Field width
        self.c = 3.0   # Penalty area length
        self.d = 2.0   # Goal area length
        self.e = 1.0   # Penalty area width
        self.f = 0.5   # Goal area width
        self.g = 0.75  # Corner arc radius
        self.h = 1.5   # Center circle diameter
        self.i = 2.0   # Penalty mark distance
        self.j = 0.1   # Center spot radius
        self.k = 0.1   # Line width

class FloorMap:
    def __init__(self):
        self._model = FieldModel()
        self._ppm = 100.0  # pixels per meter
        self._borderSize = 1.0
        self._originX = 0.0
        self._originY = 0.0
        self._numPixelsX = 0
        self._numPixelsY = 0
        self._sizeX = 0.0
        self._sizeY = 0.0

    def configure(self, pixelsPerMeter, borderSize):
        self._ppm = pixelsPerMeter
        self._borderSize = borderSize
        self._sizeX = self._model.b + 2.0 * self._borderSize
        self._sizeY = self._model.a + 2.0 * self._borderSize
        self._originX = -0.5 * self._sizeX
        self._originY = -0.5 * self._sizeY
        self._numPixelsX = int(self._sizeX * self._ppm)
        self._numPixelsY = int(self._sizeY * self._ppm)

    def setModel(self, model):
        self._model = model

    def pointToPixel(self, p):
        # Returns (col, row)
        col = int((p.y - self._originY) * self._ppm)
        row = int((p.x - self._originX) * self._ppm)
        return (col, row)

    def create_floor_map(self):
        if self._numPixelsX == 0 or self._numPixelsY == 0:
            raise RuntimeError("Invalid floor map dimensions")
        # Changed: swap dimensions for horizontal background
        mat = np.zeros((self._numPixelsX, self._numPixelsY), dtype=np.uint8)
        color = (255,)
        lw = int(self._model.k * self._ppm)

        # Draw Field Boundary (rectangle)
        center = Point(0, 0)
        size = Point(self._model.b - self._model.k, self._model.a - self._model.k)
        top_left = self.pointToPixel(center - size * 0.5)
        bottom_right = self.pointToPixel(center + size * 0.5)
        cv2.rectangle(mat, top_left, bottom_right, color, lw)

        # Middle Line
        p_from = self.pointToPixel(Point(-0.5 * (self._model.b - self._model.k), 0))
        p_to = self.pointToPixel(Point(0.5 * (self._model.b - self._model.k), 0))
        cv2.line(mat, p_from, p_to, color, lw)

        # Center Circle
        if self._model.h > 0.1:
            center_px = self.pointToPixel(Point(0, 0))
            radius = int(0.5 * (self._model.h - self._model.k) * self._ppm)
            cv2.circle(mat, center_px, radius, color, lw)

        # Penalty and Goal Areas
        for sign in [-1, 1]:
            # Penalty Area
            if self._model.c > 0.0 and self._model.e > 0.0:
                center_pen = Point(0, sign * 0.5 * (self._model.a - self._model.e))
                size_pen = Point(self._model.c - self._model.k, self._model.e - self._model.k)
                tl = self.pointToPixel(center_pen - size_pen * 0.5)
                br = self.pointToPixel(center_pen + size_pen * 0.5)
                cv2.rectangle(mat, tl, br, color, lw)
            # Goal Area
            if self._model.d > 0.0 and self._model.f > 0.0:
                center_goal = Point(0, sign * 0.5 * (self._model.a - self._model.f))
                size_goal = Point(self._model.d - self._model.k, self._model.f - self._model.k)
                tl = self.pointToPixel(center_goal - size_goal * 0.5)
                br = self.pointToPixel(center_goal + size_goal * 0.5)
                cv2.rectangle(mat, tl, br, color, lw)

        # Corner Arcs
        if self._model.g > 0.0:
            for signX in [-1, 1]:
                for signY in [-1, 1]:
                    center_arc = Point(signX * 0.5 * self._model.b, signY * 0.5 * self._model.a)
                    center_px = self.pointToPixel(center_arc)
                    axes = (int((self._model.g - 0.5 * self._model.k) * self._ppm),
                            int((self._model.g - 0.5 * self._model.k) * self._ppm))
                    # Determine angles based on quadrant
                    if signX > 0 and signY > 0:
                        startAngle, endAngle = 5, 85
                        angle = 180
                    elif signX > 0 and signY < 0:
                        startAngle, endAngle = 5, 85
                        angle = 270
                    elif signX < 0 and signY > 0:
                        startAngle, endAngle = 5, 85
                        angle = 90
                    else:
                        startAngle, endAngle = 5, 85
                        angle = 0
                    cv2.ellipse(mat, center_px, axes, angle, startAngle, endAngle, color, lw)

        # Center Spot
        if self._model.j > 0.0:
            center_px = self.pointToPixel(Point(0, 0))
            cv2.circle(mat, center_px, 1, color, int(2 * self._model.j * self._ppm))
        
        # Penalty Spots
        if self._model.i > 0.0:
            for sign in [-1, 1]:
                spot_center = Point(0, sign * (0.5 * self._model.a - self._model.i))
                center_px = self.pointToPixel(spot_center)
                cv2.circle(mat, center_px, 1, color, int(2 * self._model.j * self._ppm))
        
        return mat

    def blur_single_pass(self, image, whitePixels, blurFactor, blurMinValue):
        ny, nx = image.shape
        numAdded = 0
        whitePixelsCurrent = list(whitePixels)
        for (x, y) in whitePixelsCurrent:
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    neighborX = x + i
                    neighborY = y + j
                    if image[neighborY, neighborX] == 0:
                        newValue = int(blurFactor * image[y, x])
                        if newValue > blurMinValue:
                            if 0 < neighborX < nx-1 and 0 < neighborY < ny-1:
                                image[neighborY, neighborX] = newValue
                                whitePixels.append((neighborX, neighborY))
                                numAdded += 1
        # Remove current pixels from list
        del whitePixels[:len(whitePixelsCurrent)]
        return numAdded

    def apply_blur(self, image, blurFactor=0.7, blurMaxDepth=10, blurMinValue=30):
        ny, nx = image.shape
        whitePixels = [(x, y) for x in range(1, nx-1) for y in range(1, ny-1) if image[y, x] > 0]
        imageOut = image.copy()
        iteration = 0
        done = False
        while not done and iteration < blurMaxDepth:
            added = self.blur_single_pass(imageOut, whitePixels, blurFactor, blurMinValue)
            done = (added == 0)
            iteration += 1
        return imageOut

if __name__ == "__main__":
    try:
        floorMap = FloorMap()
        floorMap.configure(100.0, 1.0)  # 100 pixels per meter, 1m border
        
        # Optionally customize the field model:
        # model = FieldModel()
        # model.a = 24.0; model.b = 8.0
        # floorMap.setModel(model)
        
        # Generate floor map and apply blur
        map_img = floorMap.create_floor_map()
        blurred_img = floorMap.apply_blur(map_img)
        
        # Display results
        cv2.imshow("Original Floor Map", map_img)
        cv2.imshow("Blurred Floor Map", blurred_img)
        cv2.waitKey(0)
    except Exception as e:
        print("Error:", e)
