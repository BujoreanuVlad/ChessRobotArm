import serial
import sys


if len(sys.argv) != 3:
    sys.exit(-1)

motor_code = sys.argv[1]

while motor_code not in ['C', 'W', 'E', 'S']:
    sys.exit(1)

angle = sys.argv[2]

try:
    angle = int(angle)
    if not (angle >= 0 and angle <= 180):
        sys.exit(3)
except ValueError:
    sys.exit(2)

full_instruction = motor_code + str(angle)
print('Full instruction is:', full_instruction)

s = serial.Serial('/dev/ttyACM0', 115200)

if s.isOpen():
    s.close()
s.open()

s.write(full_instruction.encode())
response = s.readline()
print(response.decode())

s.close()
