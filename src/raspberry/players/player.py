from abc import ABC, abstractmethod
from typing import List
from ..chessBoard import ChessBoard

class Player(ABC):

    def __init__(self, side: str="white"):

        self.side = side

    @abstractmethod
    def playMove(self, board: ChessBoard) -> None:
        pass
