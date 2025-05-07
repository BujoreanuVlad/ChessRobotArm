from abc import ABC, abstractmethod

class Player(ABC):

    def __init__(self, side="white"):

        self.side = side


    @abstractmethod
    def playMove(self, board):
        pass
