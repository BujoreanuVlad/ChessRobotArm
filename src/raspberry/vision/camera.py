from libcamera import Transform
from picamera2 import Picamera2
import cv2

class Camera:

    def __new__(cls):
        if not hasattr(cls, 'singleton_instance'):
            cls.singleton_instance = super(Camera, cls).__new__(cls)
            cls.singleton_instance.camera = Picamera2()
            config = cls.singleton_instance.camera.create_still_configuration(transform=Transform(hflip=True, vflip=True))
            cls.singleton_instance.camera.configure(config)
            cls.singleton_instance.camera.start()

        return cls.singleton_instance
    
    def getFrame(self):
        return self.camera.capture_array()

    def getCVFrame(self):
        frame = self.getFrame()
        cvFrame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return cvFrame

    def __del__(self):
        self.camera.stop()
