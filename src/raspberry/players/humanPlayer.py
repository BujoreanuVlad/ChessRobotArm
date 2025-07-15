from typing import List
from .player import Player
from ..vision.boardVision import BoardVisionModule
from ..vision.camera import Camera
from ..vision.pieceRecognizer import PieceRecognizer
from ..chessBoard import ChessBoard
import time

class HumanPlayer(Player):

    def __init__(self, side: str):
        super().__init__(side)
        self.camera = Camera()
        self.vision = BoardVisionModule()
        self.pieceRecognizer = PieceRecognizer()

    def getDifferences(self, board: ChessBoard, predictedBoard: List[List[int]]):

        diffs = []

        for i in range(8):
            for j in range(8):
                if board.board[i][j] != predictedBoard[i][j]:
                    diffs.append((i, j))
        return diffs

    def applyVariation(board: ChessBoard, variation):
        newBoard = deepcopy(board.board)
        for i, j, val in variation:
            newBoard[i][j] = val
        return newBoard

    def tryAlternativePieces(prevBoard, confidences, emptySquare, uncertainSquare):

        possiblePieces = [ChessBoard.PAWN_CODE, ChessBoard.KNIGHT_CODE, ChessBoard.BISHOP_CODE, ChessBoard.ROOK_CODE, ChessBoard.QUEEN_CODE, ChessBoard.KING_CODE]
        possiblePieces = [possiblePiece * (-1 if self.side == "black" else 1) for possiblePiece in possiblePieces]

        iEmpty, jEmpty = emptySquare
        iTarget, jTarget = uncertainSquare

        confidentBoard = deepcopy(prevBoard)
        confidentBoard[iEmpty][jEmpty] = 0 

        boards = []

        for pieceCode in possiblePieces:
            confidentBoard[iTarget][jTarget] = pieceCode
            cb = ChessBoard(prevBoard)
            if cb.isMoveLegal(confidentBoard, self.side):
                boards.append((deepcopy(confidentBoard), confidences[iTarget][jTarget]))

        if len(boards) == 0:
            return None

        boards.sort(key=lambda x: -x[1])

        return board[0][0]


    def playMove(self, board: ChessBoard) -> None:
       
        print("Getting frame")
        frame = self.camera.getCVFrame()
        print("Got frame")
        warped = self.vision.getWarpedImage(frame)
        print("Got warped image")
        predictions = self.pieceRecognizer.predictFrame(warped)
        print("Got predictions")

        predictedBoard = []
       
        for row in predictions:

            predictedRow = []

            for piecePrediction in row:
                predictedPiece = self.pieceRecognizer.getMaxPrediction(piece)
                predictedRow.append(predictedPiece)

            predictedBoard.append(predictedRow)

        ChessBoard(predictedBoard, board.boardOrientation).printBoard()

        # Player hasn't made a move yet
        while predictedBoard == board.board or board.isMoveLegal(predictedBoard) == False:

            diffs = self.getDifferences(board, predictedBoard)

            if len(diffs) == 2:
                i1, j1 = diffs[0]
                i2, j2 = diffs[1]
                
                candidates = None

                if board.board[i1][j1] == 0:
                    candidates = self.tryAlternativePieces(board.board, predictions, (i1, j1), (i2, j2))
                    
                elif board.board[i2][j2] == 0:
                    candidates = self.tryAlternativePieces(board.board, predictions, (i2, j2), (i1, j1))

                if candidates is not None:
                    board.board = candidates
                    return

            candidates = [(i, j, predictedBoard[i][j], predictions[i][j]) for (i, j) in diffs if predictions[i][j] < 0.9]
            
            bestBoards = []

            for cells in itertools.combinations(candidates, r=2):
                variation = []
                for (i, j, newVal, confidence) in cells:
                    variation.append((i, j, newVal))
                newBoard = applyVariation(board, variation)
                if board.isMoveLegal(newBoard, side):
                    score = sum(predictions[i][j] for i, j, _ in variation)
                    bestBoards.append((newBoard, score))

            if len(bestBoards) > 0:
                bestBoards.sort(key=lambda x: -x[1])
                predictedBoard = bestBoards[0][0]
                board.board = predictedBoard
                return

            time.sleep(10)

            frame = self.camera.getCVFrame()
            warped = self.vision.getWarpedImage(frame)
            predictions = self.pieceRecognizer.predictFrame(warped)
            print("Got frame")

            predictedBoard = []
           
            for row in predictions:

                predictedRow = []

                for piecePrediction in row:
                    predictedPiece = self.pieceRecognizer.getMaxPrediction(piece)
                    predictedRow.append(predictedPiece)

                predictedBoard.append(predictedRow)

            ChessBoard(predictedBoard, board.boardOrientation).printBoard()

        # A legal move has been made
        board.board = predictedBoard





