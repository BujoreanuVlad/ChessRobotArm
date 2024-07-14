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


print('Full instruction is:', motor_code + str(angle))