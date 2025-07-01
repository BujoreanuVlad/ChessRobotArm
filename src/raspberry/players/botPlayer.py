from .robotPlayer import RobotPlayer
from ..chessBoard import ChessBoard 
from typing import List, Tuple
import numpy as np

class BotPlayer(RobotPlayer):

    MAX_DEPTH = 4
    MAX = ChessBoard.KING_CODE
    MIN = -MAX

    def getMaterialScore(board: List[List[int]]) -> float:
       
        materialScore = 0

        for i in range(8):
            for j in range(8):
                materialScore += board[i][j]

        return materialScore

    def getPlacementScore(board: List[List[int]]) -> float:

        placementScore = 0

        for i in range(3, 5):
            for j in range(3, 5):
                placementScore += np.sign(board[i][j]) / 2

        for i in [2, 5]:
            for j in range(2, 6):
                placementScore += np.sign(board[i][j]) / 4


        for i in range(3, 5):
            for j in [2, 5]:
                placementScore += np.sign(board[i][j]) / 4

        return placementScore

    def getPawnAdvancementScore(board: List[List[int]], boardOrientation: str) -> float:

        pawnAdvancementScore = 0
        advancementTilePoints = 0.25

        boardOrientationSign = 1

        if boardOrientation == "reversed":
            boardOrientationSign = -1

        for i in range(8):
            for j in range(8):
                if abs(board[i][j]) == ChessBoard.PAWN_CODE:
                    if np.sign(board[i][j]) > 0:
                        pawnAdvancementScore += advancementTilePoints * i if boardOrientationSign == -1 else advancementTilePoints * (7-i)
                    else:
                        pawnAdvancementScore -= advancementTilePoints * i if boardOrientationSign == 1 else advancementTilePoints * (7-i)


        return pawnAdvancementScore


    def getMobilityScore(board: List[List[int]], boardOrientation: str) -> float:

        mobilityScore = 0

        boardOrientationSign = 1

        if boardOrientation == "reversed":
            boardOrientationSign = -1

        return mobilityScore


    def evaluatePosition(board: List[List[int]], boardOrientation: str) -> float:

        positionScore = 0

        materialScore = BotPlayer.getMaterialScore(board)
        positionScore += materialScore

        placementScore = BotPlayer.getPlacementScore(board)
        positionScore += placementScore

        pawnAdvancementScore = BotPlayer.getPawnAdvancementScore(board, boardOrientation)
        positionScore += pawnAdvancementScore

        mobilityScore = BotPlayer.getMobilityScore(board, boardOrientation)
        positionScore += mobilityScore

        return positionScore

    
    def alphaBetaPruning(board: ChessBoard, depth: int, maximizingPlayer: bool, alpha: float, beta: float) -> Tuple[float, List[List[int]]]:

        # Check if node is leaf node

        if board.isDraw("white" if maximizingPlayer else "black"):
            return 0

        if maximizingPlayer:
            if board.isCheckMate("white"):
                return -ChessBoard.KING_CODE, board.board
        else:
            if board.isCheckMate("black"):
                return ChessBoard.KING_CODE, board.board

        if depth >= BotPlayer.MAX_DEPTH:
            return BotPlayer.evaluatePosition(board.board, board.boardOrientation), board.board

        
        if maximizingPlayer:

            best = BotPlayer.MIN
            bestMove = None

            whiteMoves = board.getLegalMoves("white")
            whiteMoves = np.random.permutation(whiteMoves)

            for move in whiteMoves:

                value, _ = BotPlayer.alphaBetaPruning(ChessBoard(move), depth+1, False, alpha, beta)

                if best < value:
                    best = value
                    bestMove = move

                alpha = max(alpha, best)

                # Prune
                if beta <= alpha:
                    break

            return best, bestMove

        else:

            best = BotPlayer.MAX
            bestMove = None

            blackMoves = board.getLegalMoves("black")
            blackMoves = np.random.permutation(blackMoves)

            for move in blackMoves:

                value, _ = BotPlayer.alphaBetaPruning(ChessBoard(move), depth+1, True, alpha, beta)

                if best > value:
                    best = value
                    bestMove = move

                beta = min(beta, best)

                # Pruning
                if beta <= alpha:
                    break

            return best, bestMove


    def computeMove(self, board: ChessBoard) -> Tuple[int, int, int, int, int]:
        
        best, bestMove = BotPlayer.alphaBetaPruning(board, 0, self.side == "white", BotPlayer.MIN, BotPlayer.MAX)
        return board.getDifference(bestMove)

