from .robotPlayer import RobotPlayer
from ..game import Game
from typing import List, Tuple

class BotPlayer(RobotPlayer):

    def computeMove(self, board: List[List[int]], boardOrientation: str) -> Tuple[int, int, int, int, int]:
        
        sideSign = 1
        orientationSign = 1

        if self.side == "black":
            sideSign = -1

        if boardOrientation == "reversed":
            orientationSign = -1

        for i in range(len(board)):
            for j in range(len(board[i])):

                # Found piece for this player's side
                if sideSign * board[i][j] > 0:

                    # Piece is a pawn
                    if board[i][j] == sideSign * Game.PAWN_CODE:
                        # If there is an empty space in front of the pawn, move the pawn
                        if board[i + sideSign * orientationSign][j] == 0:
                            # If it's the end of the board promote the pawn to a queen
                            if i + sideSign * orientationSign == 0 or \
                                i + sideSign * orientationSign == len(board):

                                board[i + sideSign * orientationSign][j] = Game.QUEEN_CODE
                                return j+1, i+1, j+1, i+1 + sideSign * orientationSign, Game.QUEEN_CODE * sideSign
                            return j+1, i+1, j+1, i+1 + sideSign * orientationSign, 0
                        

