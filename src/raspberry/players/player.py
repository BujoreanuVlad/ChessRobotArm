from abc import ABC, abstractmethod
from typing import List

class Player(ABC):

    def __init__(self, side: str="white"):

        self.side = side

    def playMove(self, board: List[List[int]], boardOrientation: str) -> None:
        pass
