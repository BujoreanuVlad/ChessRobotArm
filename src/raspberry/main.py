from arduinoController import ArduinoController
from robotState import RobotState


robotState = RobotState()
arduinoController = ArduinoController()

boardMeasurements = [0, 0, 0, 0]

robotState.registerAction(RobotState.CALIBRATE_ACTION, lambda x: arduinoController.calibrate(boardMeasurements[0], boardMeasurements[1], boardMeasurements[2], boardMeasurements[3]))

robotState.setSideLED(RobotState.SIDE_WHITE_LED_PIN)
robotState.setStateLED(RobotState.STATE_FINISHED_LED_PIN)
