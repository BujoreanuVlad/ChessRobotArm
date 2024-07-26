import serial
import time

s = serial.Serial('/dev/ttyACM0', 115200)
if s.isOpen():
    print("Port is already open, closing")
    s.close()
s.open()
time.sleep(1.75)

s.write("150\n".encode())

try:
    while True:
        response = s.readline()
        print(response.decode())
except KeyboardInterrupt:
    s.close()
print("Closed connection")
