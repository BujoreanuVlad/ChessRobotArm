from abc import ABC, abstractmethod
from typing import List
from ..chessBoard import ChessBoard

class Player(ABC):

    def __init__(self, side: str="white"):

        self.side = side

    def playMove(self, board: ChessBoard, boardOrientation: str) -> None:
        pass
