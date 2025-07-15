from libcamera import Transform
from picamera2 import Picamera2

class Camera:

    def __new__(cls):
        if not hasattr(cls, 'singleton_instance'):
            cls.singleton_instance = super(Camera, cls).__new__(cls)
            cls.singleton_instance.camera = Picamera2()
            config = cls.singleton_instance.camera.create_still_configuration(transform=Transform(hflip=True, vflip=True))
            cls.singleton_instance.camera.configure(config)
            cls.singleton_instance.camera.start()
            if not cls.singleton_instance.arduino.isOpen():
                cls.singleton_instance.arduino.open()

        return cls.singleton_instance
    
    def getFrame(self):
        return self.camera.capture_array()

    def __del__(self):
        self.camera.stop()
