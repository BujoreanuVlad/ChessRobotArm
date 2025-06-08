from .player import Player
from ..arduinoController import ArduinoController
from abc import abstractmethod
from typing import List, Tuple

class RobotPlayer(Player):

    def __init__(self, side: str="white"):
        super().__init__(side)
        self.arduinoController = ArduinoController()

    def playMove(self, board: List[List[int]], boardOrientation: str) -> None:

        initColumn, initLine, finalColumn, finalLine, transformationCode = computeMove(board, boardOrientation)

        # TODO: if piece takes the place of another, make a capture piece function
        if board[finalLine-1][finalColumn-1] != 0:
            pass

        # TODO: if piece moved is a pawn that needs to be transformed, make a pawn promotion function
        if transformationCode != 0:
            pass
        else:
            arduinoController.movePiece(initColumn, initLine, finalColumn, finalLine)
            board[finalLine-1][finalColumn-1] = board[initLine-1][initColumn-1]

        board[initLine-1][initColumn-1] = 0

    @abstractmethod
    def computeMove(self, board: List[List[int]], boardOrientation: str) -> Tuple[int, int, int, int, int]:
        pass
    
