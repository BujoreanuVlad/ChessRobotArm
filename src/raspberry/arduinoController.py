import serial
from .vision.boardVision import BoardVisionModule
from .vision.camera import Camera

class ArduinoController(object):

    def __new__(cls):
        if not hasattr(cls, 'singleton_instance'):
            cls.singleton_instance = super(ArduinoController, cls).__new__(cls)
            cls.singleton_instance.arduino = serial.Serial('/dev/ttyACM0', 115200)
            if not cls.singleton_instance.arduino.isOpen():
                cls.singleton_instance.arduino.open()

        return cls.singleton_instance

    def getXOffsetFromCorners(self, corners, imageHeight: int):
        distanceInPixels = (corners[1][1] + corners[3][1]) / 2
        distanceInPixels = imageHeight - distanceInPixels
        x = distanceInPixels
        return 2.004e-05 * (x**2) - 0.04459 * x + 66.47

    def calibrate(self):
        self.arduino.write("c\n".encode('utf-8'))
        #Wait to finish calibration
        self.arduino.readline()

        camera = Camera()
        frame = camera.getFrame()
        visionModule = BoardVisionModule()
        corners = visionModule.getCornersFromPicamFrame(frame)

        boardXOffset = self.getXOffsetFromCorners(corners, len(frame))
        boardYOffset = 0
        boardLength = 31.6
        boardHeight = 2

        self.arduino.write(('i'+str(boardLength)+";"+str(boardXOffset)+";"+str(boardYOffset)+";"+str(boardHeight)+"\n").encode("utf-8"))

    def movePiece(self, initialColumn: int, initialLine: int, finalColumn: int, finalLine: int):
        self.arduino.write(('b' + str(initialColumn) + str(initialLine) + str(finalColumn) + str(finalLine)+"\n").encode('utf-8'))

    def __del__(self):
        self.arduino.close()
