from .players.player import Player

class Game:

    PAWN_CODE = 1
    KNIGHT_CODE = 2
    BISHOP_CODE = 3
    ROOK_CODE = 5
    QUEEN_CODE = 10
    KING_CODE = 100000000

    # boardOrientation = normal (white bottom, black top), reversed (black bottom, white top)
    def __init__(self, player1: Player, player2: Player, board=None, boardOrientation: str="normal"):
        self.state = "paused"
        self.winner = None

        if player1.side == "white":
            self.player1 = player1
            self.player2 = player2
        else: 
            self.player1 = player2
            self.player2 = player1

        self.boardOrientation = "normal"

        if board is None:
            self.board = [[0] * 8 for i in range(8)]
            self.board[0][0] = Game.ROOK_CODE
            self.board[0][1] = Game.KNIGHT_CODE
            self.board[0][2] = Game.BISHOP_CODE
            self.board[0][3] = Game.QUEEN_CODE
            self.board[0][4] = Game.KING_CODE
            self.board[0][5] = Game.BISHOP_CODE
            self.board[0][6] = Game.KNIGHT_CODE
            self.board[0][7] = Game.ROOK_CODE
            self.board[1] = [Game.PAWN_CODE] * 8

            self.board[-1] = [-piece for piece in self.board[0]]
            self.board[-2] = [-piece for piece in self.board[1]]

            self.board = self.board[::-1] # Have white pieces at the bottom
        else:
            self.board = board

    def playTurn(self):

        self.player1.playMove(self.board, self.boardOrientation)
        self.player2.playMove(self.board, self.boardOrientation)

    def checkGameState(self):
        pass

    def printBoard(self):
        
        for row in self.board:
            for cell in row:
                if cell < 0:
                    print("b", end='')
                elif cell > 0:
                    print('w', end='')
                else:
                    print('0'.ljust(2), end=' ')

                cell = abs(cell)

                if cell == Game.PAWN_CODE:
                    print("p", end=' ')
                if cell == Game.BISHOP_CODE:
                    print("B", end=' ')
                if cell == Game.KNIGHT_CODE:
                    print("N", end=' ')
                if cell == Game.QUEEN_CODE:
                    print("Q", end=' ')
                if cell == Game.KING_CODE:
                    print("K", end=' ')
                if cell == Game.ROOK_CODE:
                    print("R", end=' ')
            print()
