import unittest
from ..chessBoard import ChessBoard

class ChessBoardChecks(unittest.TestCase):

    def test_whenStandardBoardCheckIfWhiteCheck(self):

        board = ChessBoard()

        isCheck = board.isCheck("white")

        self.assertEqual(isCheck, False)

    def test_whenStandardBoardCheckIfBlackCheck(self):

        board = ChessBoard()

        isCheck = board.isCheck("black")

        self.assertEqual(isCheck, False)
        
    def test_whenReversedBoardCheckIfWhiteCheck(self):

        board = ChessBoard(boardOrientation="reversed")

        isCheck = board.isCheck("white")

        self.assertEqual(isCheck, False)

    def test_whenReversedBoardCheckIfBlackCheck(self):

        board = ChessBoard(boardOrientation="reversed")

        isCheck = board.isCheck("black")

        self.assertEqual(isCheck, False)

    def test_whenWhiteChecksBlack1CheckIfBlackCheck(self):

        boardCodes = [
            [-5, -2, -3, -10, -100000000, -3, -2, -5],
            [0, -1, -1, 0, -1, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [5, 2, 3, 0, 100000000, 3, 2, 5]
        ]
        board = ChessBoard(boardCodes)

        isCheck = board.isCheck("black")

        self.assertTrue(isCheck, "When white checks black, black is not in check")

    def test_whenWhiteChecksBlack1CheckIfWhiteCheck(self):

        boardCodes = [
            [-5, -2, -3, -10, -100000000, -3, -2, -5],
            [0, -1, -1, 0, -1, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [5, 2, 3, 0, 100000000, 3, 2, 5]
        ]
        board = ChessBoard(boardCodes)

        isCheck = board.isCheck("white")

        self.assertFalse(isCheck, "When white checks black, white is in check")

    def test_whenWhiteChecksBlack1CheckIfBlackCheckMate(self):

        boardCodes = [
            [-5, -2, -3, -10, -100000000, -3, -2, -5],
            [0, -1, -1, 0, -1, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [5, 2, 3, 0, 100000000, 3, 2, 5]
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertFalse(isCheckMate, "When white only checks black, black is check mated")

    def test_whenWhiteChecksBlack2CheckIfBlackCheckMate(self):

        boardCodes = [
            [-5, -2, -3, -10, -100000000, -3, -2, -5],
            [-1, -1, -1, 0, -1, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [5, 2, 3, 0, 100000000, 3, 2, 5]
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertFalse(isCheckMate, "When white only checks black, black is checkmated")


    def test_whenWhiteChecksBlack3CheckIfBlackCheck(self):

        boardCodes = [
            [0, 0, 0, 0, 0, 0, -ChessBoard.KNIGHT_CODE, -ChessBoard.KING_CODE],
            [0, 0, 0, 0, 0, 0, ChessBoard.PAWN_CODE, -ChessBoard.KNIGHT_CODE],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, ChessBoard.KING_CODE, 0, 0, 0, 0, 0],
        ]
        board = ChessBoard(boardCodes)

        isCheck = board.isCheck("black")

        self.assertTrue(isCheck, "When white checks black with just a pawn, black is not checked")

    def test_whenWhiteChecksBlack3CheckIfBlackCheckMate(self):

        boardCodes = [
            [0, 0, 0, 0, 0, 0, -ChessBoard.KNIGHT_CODE, -ChessBoard.KING_CODE],
            [0, 0, 0, 0, 0, 0, ChessBoard.PAWN_CODE, -ChessBoard.KNIGHT_CODE],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, ChessBoard.KING_CODE, 0, 0, 0, 0, 0],
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertFalse(isCheckMate, "When white checks black with just a pawn, black is checkmated instead of capturing")

    def test_whenWhiteCheckMatesBlack4CheckIfBlackCheck(self):

        boardCodes = [
            [0, 0, 0, 0, 0, 0, -ChessBoard.KNIGHT_CODE, -ChessBoard.KING_CODE],
            [0, 0, 0, 0, 0, ChessBoard.KNIGHT_CODE, -ChessBoard.PAWN_CODE, -ChessBoard.KNIGHT_CODE],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, ChessBoard.KING_CODE, 0, 0, 0, 0, 0],
        ]
        board = ChessBoard(boardCodes)

        isCheck = board.isCheck("black")

        self.assertTrue(isCheck, "When white smother mates black with a knight, black is not checked")

    def test_whenWhiteCheckMatesBlack4CheckIfBlackCheckMate(self):

        boardCodes = [
            [0, 0, 0, 0, 0, 0, -ChessBoard.KNIGHT_CODE, -ChessBoard.KING_CODE],
            [0, 0, 0, 0, 0, ChessBoard.KNIGHT_CODE, -ChessBoard.PAWN_CODE, -ChessBoard.KNIGHT_CODE],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, ChessBoard.KING_CODE, 0, 0, 0, 0, 0],
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertTrue(isCheckMate, "When white smother mates black with a knight, black is not checkmated")

    def test_whenWhiteChecksBlack5CheckIfBlackCheckMate(self):

        boardCodes = [
            [-5, 0, -3, 0, -100000000, -3, -2, -5],
            [-1, -1, -1, 0, -1, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [5, 2, 3, 0, 100000000, 3, 2, 5]
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertFalse(isCheckMate, "When white only checks black, black is checkmated")

    def test_whenWhiteChecksBlack6CheckIfBlackCheckMate(self):

        boardCodes = [
            [-5, 0, 0, -3, -100000000, -3, -2, -5],
            [-1, -1, 0, 0, -1, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [5, 2, 3, 0, 100000000, 3, 2, 5]
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertFalse(isCheckMate, "When white only checks black, black is checkmated")

    def test_whenWhiteChecksBlack7CheckIfBlackCheckMate(self):

        boardCodes = [
            [-5, 0, 0, -3, -100000000, -3, -2, -5],
            [-1, 0, 0, 0, -1, -1, -1, -1],
            [0, -1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [5, 2, 3, 0, 100000000, 3, 2, 5]
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertFalse(isCheckMate, "When white only checks black, black is checkmated")

    def test_whenWhiteCheckMatesBlack8CheckIfBlackCheck(self):

        boardCodes = [
            [-5,  0, -3, -10, -100000000, -3, -2, -5],
            [-1, -1, -1,   0,      0,     10, -1, -1],
            [ 0,  0, -2,  -1,      0,      0,  0,  0],
            [ 0,  0,  0,   0,     -1,      0,  0,  0],
            [ 0,  0,  3,   0,      1,      0,  0,  0],
            [ 0,  0,  0,   0,      0,      0,  0,  0],
            [ 1,  1,  1,   1,      0,      1,  1,  1],
            [ 5,  2,  3,   0, 100000000,   0,  2,  5]
        ]
        board = ChessBoard(boardCodes)

        isCheck = board.isCheck("black")

        self.assertTrue(isCheck, "When white check mates black, black is not checked")

    def test_whenWhiteCheckMatesBlack8CheckIfBlackCheckMate(self):

        boardCodes = [
            [-5,  0, -3, -10, -100000000, -3, -2, -5],
            [-1, -1, -1,   0,      0,     10, -1, -1],
            [ 0,  0, -2,  -1,      0,      0,  0,  0],
            [ 0,  0,  0,   0,     -1,      0,  0,  0],
            [ 0,  0,  3,   0,      1,      0,  0,  0],
            [ 0,  0,  0,   0,      0,      0,  0,  0],
            [ 1,  1,  1,   1,      0,      1,  1,  1],
            [ 5,  2,  3,   0, 100000000,   0,  2,  5]
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertTrue(isCheckMate, "When white check mates black, black is not checkmated")

    def test_whenWhiteChecksBlack9CheckIfBlackCheck(self):

        boardCodes = [
            [-5,  0, -3, -10, -100000000, -3, -2, -5],
            [-1, -1, -1,   0,      0,     10, -1, -1],
            [ 0,  0, -2,   0,      0,      0,  0,  0],
            [ 0,  0,  0,  -1,     -1,      0,  0,  0],
            [ 0,  0,  3,   0,      1,      0,  0,  0],
            [ 0,  0,  0,   0,      0,      0,  0,  0],
            [ 1,  1,  1,   1,      0,      1,  1,  1],
            [ 5,  2,  3,   0, 100000000,   0,  2,  5]
        ]
        board = ChessBoard(boardCodes)

        isCheck = board.isCheck("black")

        self.assertTrue(isCheck, "When white checks black, black is not checked")
        
    def test_whenWhiteChecksBlack9CheckIfBlackCheckMate(self):

        boardCodes = [
            [-5,  0, -3, -10, -100000000, -3, -2, -5],
            [-1, -1, -1,   0,      0,     10, -1, -1],
            [ 0,  0, -2,   0,      0,      0,  0,  0],
            [ 0,  0,  0,  -1,     -1,      0,  0,  0],
            [ 0,  0,  3,   0,      1,      0,  0,  0],
            [ 0,  0,  0,   0,      0,      0,  0,  0],
            [ 1,  1,  1,   1,      0,      1,  1,  1],
            [ 5,  2,  3,   0, 100000000,   0,  2,  5]
        ]
        board = ChessBoard(boardCodes)

        isCheckMate = board.isCheckMate("black")

        self.assertFalse(isCheckMate, "When white checks black, black is checkmated")
    

if __name__ == '__main__':
    unittest.main()
