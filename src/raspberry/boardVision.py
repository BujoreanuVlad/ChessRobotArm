import cv2

class BoardVisionModule:

    def preprocessImage(self, image):
        
        processedImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 49, 1)

        return thresh


    def detectCorners(self, processedImage):

        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 1))
        horizontal_kernel_2 = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 1))

        vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 10))
        vertical_kernel_2 = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 40))

        morph_1_horizontal = cv2.morphologyEx(thresh_1, cv2.MORPH_OPEN, horizontal_kernel, iterations=1)
        morph_1_horizontal_2 = cv2.morphologyEx(morph_1_horizontal, cv2.MORPH_OPEN, horizontal_kernel_2, iterations=1)

        morph_1_vertical = cv2.morphologyEx(thresh_1, cv2.MORPH_OPEN, vertical_kernel, iterations=1)
        morph_1_vertical_2 = cv2.morphologyEx(morph_1_vertical, cv2.MORPH_OPEN, vertical_kernel_2, iterations=1)

        combined_or_1 = cv2.bitwise_or(morph_1_horizontal_2, morph_1_vertical_2)
        ellipse_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (81, 81))
        dilate_2 = cv2.morphologyEx(combined_or_1, cv2.MORPH_CLOSE, ellipse_kernel)

        contours, hierarchy = cv2.findContours(dilate_2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        maxArea = 0
        maxContour = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > maxArea:
                maxArea = area
                maxContour = cnt

        alpha = 0.05
        arcLength = cv2.arcLength(maxContour, True)
        
        while len(approx) != 4:
            if len(approx) < 4:
                alpha /= 2
            else:
                alpha = (1 + alpha) / 2
            epsilon = alpha * arcLength
            approx = cv2.approxPolyDP(maxContour, epsilon, True)

        corners = [coords[0] for coords in approx]

        return corners        

        
        
    def getWarpedImage(self, image):

       processedImage = self.preprocessImage(image)
       corners = self.detectCorners(processedImage)
       corners = BoardVision.getOrderedCorners(approx)

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
       

    def getOrderedCorners(corners):

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


