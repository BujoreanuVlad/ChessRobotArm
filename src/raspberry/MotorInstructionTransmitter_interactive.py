import serial

motor_code = input('Input the motor code:\nC -> Claw servo\nW -> Wrist servo\nE -> elbow servo\nS -> shoulder stepper\n')

while motor_code not in ['C', 'W', 'E', 'S']:
    print("Invalid input")
    motor_code = input('Input the motor code:\nC -> Claw servo\nW -> Wrist servo\nE -> elbow servo\nS -> shoulder stepper\n')

angle = None

while type(angle) != int or not (angle >= 0 and angle <= 180):
        
    angle = input('Input angle (0-180) in degrees: ')

    try:
        angle = int(angle)
        if not (angle >= 0 and angle <= 180):
            print("Error, enter a valid integer")
    except ValueError:
        print("Error, enter a valid integer")


print('Full instruction is:', motor_code + str(angle))