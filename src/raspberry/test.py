from .game import Game
from .players.humanPlayer import HumanPlayer
from .players.botPlayer import BotPlayer
from .chessBoard import ChessBoard
from .robotState import RobotState
from .arduinoController import ArduinoController

human = HumanPlayer("white")
bot = BotPlayer("black")

arduinoController = ArduinoController()

robotState = RobotState()
orientation = "normal" if robotState.isLEDOn(RobotState.SIDE_WHITE_LED_PIN) else "reversed"
board = ChessBoard(boardOrientation=orientation)

robotState.registerAction(RobotState.CALIBRATE_ACTION, lambda x: arduinoController.calibrate())

robotState.registerAction(RobotState.SIDE_ACTION, lambda x: robotState.setSideLED(RobotState.SIDE_BLACK_LED_PIN) if robotState.isLEDOn(RobotState.SIDE_WHITE_LED_PIN) else robotState.setSideLED(RobotState.SIDE_WHITE_LED_PIN))

robotState.registerAction(RobotState.START_ACTION, lambda x: robotState.setStateLED(RobotState.STATE_PLAYING_LED_PIN) if robotState.isLEDOn(RobotState.STATE_FINISHED_LED_PIN) or robotState.isLEDOn(RobotState.STATE_PAUSED_LED_PIN) else robotState.setStateLED(RobotState.STATE_PAUSED_LED_PIN))
robotState.registerAction(RobotState.START_ACTION, lambda x: game.pauseResume())

robotState.registerAction(RobotState.STOP_ACTION, lambda x: robotState.setStateLED(RobotState.STATE_FINISHED_LED_PIN))

robotState.setSideLED(RobotState.SIDE_WHITE_LED_PIN)
robotState.setStateLED(RobotState.STATE_FINISHED_LED_PIN)
robotState.setWinnerLED(None)

game = Game(bot, human, board)
game.playTurn()

board.printBoard()

game.playTurn()

board.printBoard()
