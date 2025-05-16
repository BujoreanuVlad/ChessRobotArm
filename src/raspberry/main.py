from arduinoController import ArduinoController
from robotState import RobotState


robotState = RobotState()
arduinoController = ArduinoController()

boardMeasurements = [0, 0, 0, 0]

robotState.registerAction(RobotState.CALIBRATE_ACTION, lambda x: arduinoController.calibrate(boardMeasurements[0], boardMeasurements[1], boardMeasurements[2], boardMeasurements[3]))

robotState.registerAction(RobotState.SIDE_ACTION, lambda x: robotState.setSideLED(RobotState.SIDE_BLACK_LED_PIN) if robotState.isLEDOn(RobotState.SIDE_WHITE_LED_PIN) else setSideLED(RobotState.SIDE_WHITE_LED_PIN))

robotState.registerAction(RobotState.SIDE_ACTION, lambda x: robotState.setStateLED(RobotState.STATE_PLAYING_LED_PIN) if robotState.isLEDOn(RobotState.STATE_FINISHED_LED_PIN) or robotState.isLEDOn(RobotState.STATE_PAUSED_LED_PIN) else setSideLED(RobotState.STATE_PAUSED_LED_PIN))

robotState.registerAction(RobotState.SIDE_ACTION, lambda x: robotState.setStateLED(RobotState.STATE_FINISHED_LED_PIN))

robotState.setSideLED(RobotState.SIDE_WHITE_LED_PIN)
robotState.setStateLED(RobotState.STATE_FINISHED_LED_PIN)
robotState.setWinnerLED(None)


