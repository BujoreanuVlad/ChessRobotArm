import serial

class ArduinoController(object):

    def __new__(cls):
        if not hasattr(cls, 'singleton_instance'):
            cls.singleton_instance = super(ArduinoController, cls).__new__(cls)
            cls.singleton_instance.arduino = serial.Serial('/dev/ttyACM0', 115200)
            cls.singleton_instance.arduino.open()

        return cls.singleton_instance

    def calibrate(self, boardLength: float, boardXOffset: float, boardYOffset: float, boardHeight: float):
        self.arduino.write("c\n".encode('utf-8'))
        self.arduino.write(('i'+str(boardLength)+";"+str(boardXOffset)+";"+str(boardYOffset)+";"+str(boardHeight)+"\n").encode("utf-8"))

    def movePiece(self, initialColumn: int, initialLine: int, finalColumn: int, finalLine: int):
        self.arduino.write(('b' + str(initialColumn) + str(initialLine) + str(finalColumn) + str(finalLine)+"\n").encode('utf-8'))

    def __del__(self):
        self.arduino.close()
