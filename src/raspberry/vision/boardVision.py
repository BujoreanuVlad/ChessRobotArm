import cv2
import numpy as np
from copy import deepcopy
from typing import List

class BoardVisionModule:

    def __new__(cls):
        if not hasattr(cls, 'singleton_instance'):
            cls.singleton_instance = super(BoardVisionModule, cls).__new__(cls)
            cls.singleton_instance.corners = None

        return cls.singleton_instance

    def preprocessImage(self, image: np.ndarray) -> np.ndarray:

        blurred = cv2.GaussianBlur(image, (3, 3), 0)

        grayImage = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        grayImage = clahe.apply(grayImage)
        
        thresh = cv2.threshold(grayImage, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        return 255-thresh

    def dilateEdges(self, preprocessedImage: np.ndarray) -> np.ndarray:

        processedImage = preprocessedImage

        horizontalKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 1))
        horizontalKernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 1))

        verticalKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 10))
        verticalKernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 40))

        morphHorizontal = cv2.morphologyEx(processedImage, cv2.MORPH_OPEN, horizontalKernel, iterations=1)
        morphHorizontal2 = cv2.morphologyEx(morphHorizontal, cv2.MORPH_OPEN, horizontalKernel2, iterations=1)

        morphVertical = cv2.morphologyEx(processedImage, cv2.MORPH_OPEN, verticalKernel, iterations=1)
        morphVertical2 = cv2.morphologyEx(morphVertical, cv2.MORPH_OPEN, verticalKernel2, iterations=1)

        combinedOr = cv2.bitwise_or(morphHorizontal2, morphVertical2)
        ellipseKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41))
        dilate = cv2.morphologyEx(combinedOr, cv2.MORPH_CLOSE, ellipseKernel)
        #combinedOrInverted = 255 - combinedOr

        return combinedOr
        

    def detectCorners(self, processedImage: np.ndarray, firstTry: bool=True) -> List[np.ndarray]:

        contours, hierarchy = cv2.findContours(processedImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        maxArea = 0
        maxContour = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > maxArea:
                maxArea = area
                maxContour = cnt

        alpha = 0.05
        lastAlpha = 1
        arcLength = cv2.arcLength(maxContour, True)
        epsilon = alpha * arcLength
        approx = cv2.approxPolyDP(maxContour, epsilon, True)
        maxIterations = 1000
        iteration = 0
        #approx = cv2.approxPolyN(maxContour, nsides=4, ensure_convex=True)
        
        while len(approx) != 4:
            if len(approx) < 4:
                lastAlpha = alpha
                alpha /= 2
            else:
                alpha = (lastAlpha + alpha) / 2
            epsilon = alpha * arcLength
            approx = cv2.approxPolyDP(maxContour, epsilon, True)
            iteration += 1

            if iteration >= maxIterations:
                if firstTry:
                    return self.detectCorners(255 - processedImage, False)
                return None

        corners = [coords[0] for coords in approx]
        corners = BoardVisionModule.getOrderedCorners(corners)
        self.corners = corners

        return corners        

    def getCornersFromPicamFrame(self, frame) -> List[np.ndarray]:

        bgrFrame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        preprocessedImage = self.preprocessImage(bgrFrame)
        processedImage = self.dilateEdges(preprocessedImage)
        corners = self.detectCorners(processedImage)

        return corners
        
    def getWarpedImage(self, image: np.ndarray) -> np.ndarray:

        preprocessedImage = self.preprocessImage(image)
        processedImage = self.dilateEdges(preprocessedImage)
        if self.corners is None:
            corners = self.detectCorners(processedImage)

        corners = deepcopy(self.corners)

        corners[0][1] = np.clip(corners[0][1] - 240, 0, None)
        corners[2][1] = np.clip(corners[2][1] - 240, 0, None)

        corners[0][0] = np.clip(corners[0][0] + 100, 0, image.shape[0], None)
        corners[2][0] = np.clip(corners[2][0] - 100, 0, None)

        height_1 = np.sqrt(((corners[0][0] - corners[1][0]) ** 2) + ((corners[0][1] - corners[1][1]) ** 2))
        height_2 = np.sqrt(((corners[2][0] - corners[3][0]) ** 2) + ((corners[2][1] - corners[3][1]) ** 2))

        width_1 = np.sqrt(((corners[0][0] - corners[2][0]) ** 2) + ((corners[0][1] - corners[2][1]) ** 2))
        width_2 = np.sqrt(((corners[1][0] - corners[3][0]) ** 2) + ((corners[1][1] - corners[3][1]) ** 2))

        max_height=max(int(height_1), int(height_2))
        max_width = max(int(width_1), int(width_2))

        input_pts = np.float32(corners)
        output_pts = np.float32([[0, 0],
                                [0, max_width],
                                [max_height , 0],
                                [max_height , max_width]])
        M = cv2.getPerspectiveTransform(input_pts,output_pts)
        out = cv2.warpPerspective(image,M,(max_height, max_width),flags=cv2.INTER_LINEAR)

        return out

    def getPieces(self, warpedImage: np.ndarray) -> List[List[np.ndarray]]:
        
        topPieces = [warpedImage[:1100, 20+i*170:70+(i+1)*190] for i in range(8)]

        pieces = [topPieces]

        for j in range(1, 8):
            rowPieces = []
            for i in range(8):
                rowPieces.append(warpedImage[550 + j*200:900 + (j+1)*200, 20 + i*170: 70 + (i+1)*190])
            pieces.append(rowPieces)

        return pieces
       

    def getOrderedCorners(corners: List[np.ndarray]) -> List[np.ndarray]:

        corners = sorted(corners, key=lambda tup: tup[0])

        if corners[0][1] > corners[1][1]:
            tmp = corners[0].copy()
            corners[0] = corners[1]
            corners[1] = tmp
        if corners[2][1] > corners[3][1]:
            tmp = corners[2].copy()
            corners[2] = corners[3]
            corners[3] = tmp

        return corners


