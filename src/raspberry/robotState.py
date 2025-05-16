import RPi.GPIO as GPIO
from abc import ABC, abstractmethod
from typing import Final, Callable

class RobotState(object):

    WINNER_WHITE_LED_PIN: Final[int] = 11
    WINNER_BLACK_LED_PIN: Final[int] = 6
    WINNER_DRAW_LED_PIN: Final[int] = 5
    STATE_PLAYING_LED_PIN: Final[int] = 8
    STATE_PAUSED_LED_PIN: Final[int] = 10
    STATE_FINISHED_LED_PIN: Final[int] = 9
    SIDE_WHITE_LED_PIN: Final[int] = 25
    SIDE_BLACK_LED_PIN: Final[int] = 7

    CALIBRATE_BUTTON_PIN: Final[int] = 24
    SIDE_BUTTON_PIN: Final[int] = 23
    START_BUTTON_PIN: Final[int] = 22
    STOP_BUTTON_PIN: Final[int] = 27

    CALIBRATE_ACTION: Final[int] = 1
    SIDE_ACTION: Final[int] = 2
    START_ACTION: Final[int] = 3
    STOP_ACTION: Final[int] = 4

    def __new__(cls):
        if not hasattr(cls, 'singleton_instance'):

            cls.singleton_instance = super(RobotState, cls).__new__(cls)

            GPIO.setmode(GPIO.BCM)

            GPIO.setup(RobotState.WINNER_WHITE_LED_PIN, GPIO.OUT)
            GPIO.setup(RobotState.WINNER_BLACK_LED_PIN, GPIO.OUT)
            GPIO.setup(RobotState.WINNER_DRAW_LED_PIN, GPIO.OUT)
            GPIO.setup(RobotState.STATE_PLAYING_LED_PIN, GPIO.OUT)
            GPIO.setup(RobotState.STATE_PAUSED_LED_PIN, GPIO.OUT)
            GPIO.setup(RobotState.STATE_FINISHED_LED_PIN, GPIO.OUT)
            GPIO.setup(RobotState.SIDE_WHITE_LED_PIN, GPIO.OUT)
            GPIO.setup(RobotState.SIDE_BLACK_LED_PIN, GPIO.OUT)

            GPIO.setup(RobotState.CALIBRATE_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(RobotState.SIDE_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(RobotState.START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(RobotState.STOP_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

            GPIO.output(RobotState.WINNER_WHITE_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_BLACK_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_DRAW_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_PLAYING_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_PAUSED_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_FINISHED_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.SIDE_WHITE_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.SIDE_BLACK_LED_PIN, GPIO.LOW)

        return cls.singleton_instance

    def registerAction(self, button: int, buttonFunction: Callable[[int], None]):
        
        if button == RobotState.CALIBRATE_ACTION:
            GPIO.add_event_detect(RobotState.CALIBRATE_BUTTON_PIN, GPIO.RISING, callback=buttonFunction, bouncetime=200)
        elif button == RobotState.SIDE_ACTION:
            GPIO.add_event_detect(RobotState.SIDE_BUTTON_PIN, GPIO.RISING, callback=buttonFunction, bouncetime=200)
        elif button == RobotState.START_ACTION:
            GPIO.add_event_detect(RobotState.START_BUTTON_PIN, GPIO.RISING, callback=buttonFunction, bouncetime=200)
        elif button == RobotState.STOP_ACTION:
            GPIO.add_event_detect(RobotState.STOP_BUTTON_PIN, GPIO.RISING, callback=buttonFunction, bouncetime=200)


    def setSideLED(self, sidePin: int):
        
        if sidePin is None:
            GPIO.output(RobotState.SIDE_WHITE_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.SIDE_BLACK_LED_PIN, GPIO.LOW)

        elif sidePin == RobotState.SIDE_WHITE_LED_PIN:
            GPIO.output(RobotState.SIDE_WHITE_LED_PIN, GPIO.HIGH)
            GPIO.output(RobotState.SIDE_BLACK_LED_PIN, GPIO.LOW)

        elif sidePin == RobotState.SIDE_BLACK_LED_PIN:
            GPIO.output(RobotState.SIDE_WHITE_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.SIDE_BLACK_LED_PIN, GPIO.HIGH)
    
    def setStateLED(self, statePin: int):
        
        if statePin is None:
            GPIO.output(RobotState.STATE_PLAYING_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_PAUSED_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_FINISHED_LED_PIN, GPIO.LOW)

        elif statePin == RobotState.STATE_PLAYING_LED_PIN:
            GPIO.output(RobotState.STATE_PLAYING_LED_PIN, GPIO.HIGH)
            GPIO.output(RobotState.STATE_PAUSED_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_FINISHED_LED_PIN, GPIO.LOW)

        elif statePin == RobotState.STATE_PAUSED_LED_PIN:
            GPIO.output(RobotState.STATE_PLAYING_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_PAUSED_LED_PIN, GPIO.HIGH)
            GPIO.output(RobotState.STATE_FINISHED_LED_PIN, GPIO.LOW)

        elif statePin == RobotState.STATE_FINISHED_LED_PIN:
            GPIO.output(RobotState.STATE_PLAYING_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_PAUSED_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.STATE_FINISHED_LED_PIN, GPIO.HIGH)

    def setWinnerLED(self, winnerPin: int):

        if winnerPin is None:
            GPIO.output(RobotState.WINNER_WHITE_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_BLACK_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_DRAW_LED_PIN, GPIO.LOW)

        elif winnerPin == RobotState.WINNER_WHITE_LED_PIN:
            GPIO.output(RobotState.WINNER_WHITE_LED_PIN, GPIO.HIGH)
            GPIO.output(RobotState.WINNER_BLACK_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_DRAW_LED_PIN, GPIO.LOW)

        elif winnerPin == RobotState.WINNER_BLACK_LED_PIN:
            GPIO.output(RobotState.WINNER_WHITE_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_BLACK_LED_PIN, GPIO.HIGH)
            GPIO.output(RobotState.WINNER_DRAW_LED_PIN, GPIO.LOW)

        elif winnerPin == RobotState.WINNER_DRAW_LED_PIN:
            GPIO.output(RobotState.WINNER_WHITE_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_BLACK_LED_PIN, GPIO.LOW)
            GPIO.output(RobotState.WINNER_DRAW_LED_PIN, GPIO.HIGH)

    def isLEDOn(self, ledPin: int):
        
        state = GPIO.input(ledPin)

        return state


    def __del__(self):
        print("Destructor")




GPIO.cleanup()
