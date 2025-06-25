from .chessBoard import ChessBoard
from .players.player import Player

class Game:

    # boardOrientation = normal (white bottom, black top), reversed (black bottom, white top)
    def __init__(self, player1: Player, player2: Player, board: ChessBoard=None):

        self.state = "paused"
        self.winner = None

        if player1.side == "white":
            self.player1 = player1
            self.player2 = player2
        else: 
            self.player1 = player2
            self.player2 = player1


        if board is None:
            self.board = ChessBoard()
        else:
            self.board = board

    def playTurn(self):

        
        if self.board.isCheckMate("white"):
            self.state = "finished"
            self.winner = "white"
        self.player1.playMove(self.board, self.boardOrientation)
        self.player2.playMove(self.board, self.boardOrientation)
