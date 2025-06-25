from typing import List, Tuple

class ChessBoard:

    PAWN_CODE = 1
    KNIGHT_CODE = 2
    BISHOP_CODE = 3
    ROOK_CODE = 5
    QUEEN_CODE = 10
    KING_CODE = 100000000

    # boardOrientation = normal (white bottom, black top), reversed (black bottom, white top)
    def __init__(self, board=None, boardOrientation: str="normal"):

        self.boardOrientation = boardOrientation

        if board is None:
            self.board = [[0] * 8 for i in range(8)]
            self.board[0][0] = ChessBoard.ROOK_CODE
            self.board[0][1] = ChessBoard.KNIGHT_CODE
            self.board[0][2] = ChessBoard.BISHOP_CODE
            self.board[0][3] = ChessBoard.QUEEN_CODE
            self.board[0][4] = ChessBoard.KING_CODE
            self.board[0][5] = ChessBoard.BISHOP_CODE
            self.board[0][6] = ChessBoard.KNIGHT_CODE
            self.board[0][7] = ChessBoard.ROOK_CODE
            self.board[1] = [ChessBoard.PAWN_CODE] * 8

            self.board[-1] = [-piece for piece in self.board[0]]
            self.board[-2] = [-piece for piece in self.board[1]]

            if boardOrientation == "normal":
                self.board = self.board[::-1] # Have white pieces at the bottom
        else:
            self.board = board

    def _isCheckForMoveMap(self, kingLine: int, kingColumn: int, sideSign: int, moveMap: Tuple[Tuple[int, int]], pieceCode: int) -> bool:

        for direction in moveMap:

            i, j = kingLine + direction[0], kingColumn + direction[1]

            while i >= 0 and j >= 0 and i < 8 and j < 8:
                
                if self.board[i][j] != 0:
                    if self.board[i][j] * sideSign == -pieceCode:
                        return True
                    break

                i += direction[0]
                j += direction[1]

        return False

    def _isBishopCheck(self, kingLine: int, kingColumn: int, sideSign: int) -> bool:

        moveMap = ((1, 1), (1, -1), (-1, -1), (-1, 1))

        return self._isCheckForMoveMap(kingLine, kingColumn, sideSign, moveMap, ChessBoard.BISHOP_CODE)

    def _isRookCheck(self, kingLine: int, kingColumn: int, sideSign: int) -> bool:

        moveMap = ((1, 0), (-1, 0), (0, 1), (0, -1))

        return self._isCheckForMoveMap(kingLine, kingColumn, sideSign, moveMap, ChessBoard.ROOK_CODE)

    def _isQueenCheck(self, kingLine: int, kingColumn: int, sideSign: int) -> bool:

        # Define all the directions (diagonals and lines)
        moveMap = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))

        return self._isCheckForMoveMap(kingLine, kingColumn, sideSign, moveMap, ChessBoard.QUEEN_CODE)

    def _isKnightCheck(self, kingLine: int, kingColumn: int, sideSign: int) -> bool:

        moveMap = ((2, 1), (2, -1), (-2, 1), (-2, -1), (1, 2), (-1, 2), (1, -2), (-1, -2))

        for direction in moveMap:

            i = kingLine + direction[0]
            j = kingColumn + direction[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if self.board[i][j] * sideSign == -ChessBoard.KNIGHT_CODE:
                    return True

        return False

    def _isPawnCheck(self, kingLine: int, kingColumn: int, sideSign: int) -> bool:
        
        boardOrientationSign = 1

        if self.boardOrientation == "reversed":
            boardOrientationSign = -1

        i, j = kingLine - sideSign * boardOrientationSign, kingColumn

        if i >= 0 and i < 8:

            if j + 1 < 8:
                if self.board[i][j+1] * sideSign == -ChessBoard.PAWN_CODE:
                    return True

            if j - 1 >= 0:
                if self.board[i][j-1] * sideSign == -ChessBoard.PAWN_CODE:
                    return True

        return False

        
    def isCheck(self, side: str) -> bool:
        
        sideSign = 1

        if side == "black":
            sideSign = -1

        kingLine = 0
        kingColumn = 0

        for i in range(8):
            for j in range(8):
                if self.board[i][j] * sideSign == ChessBoard.KING_CODE:
                    kingLine = i
                    kingColumn = j
                    break

        return self._isBishopCheck(kingLine, kingColumn, sideSign) or \
                self._isRookCheck(kingLine, kingColumn, sideSign) or \
                self._isQueenCheck(kingLine, kingColumn, sideSign) or \
                self._isKnightCheck(kingLine, kingColumn, sideSign) or \
                self._isPawnCheck(kingLine, kingColumn, sideSign)

    def _getAttackingPieces(self, kingLine: int, kingColumn: int, sideSign: int) -> List[Tuple[int, int, int]]:
        
        knightMoveMap = ((2, 1), (2, -1), (-2, 1), (-2, -1), (1, 2), (-1, 2), (1, -2), (-1, -2))
        rookMoveMap = ((1, 0), (0, 1), (-1, 0), (0, -1))
        bishopMoveMap = ((1, 1), (-1, 1), (-1, -1), (1, -1))
        kingMoveMap = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))

        boardOrientationSign = 1

        if self.boardOrientation == "reversed":
            boardOrientationSign = -1

        attackingPieces = []

        for direction in knightMoveMap:

            i = kingLine + direction[0]
            j = kingColumn + direction[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):
                if self.board[i][j] * sideSign == -ChessBoard.KNIGHT_CODE:
                    attackingPieces.append((i, j, ChessBoard.KNIGHT_CODE))

        for direction in kingMoveMap:

            i = kingLine + direction[0]
            j = kingColumn + direction[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):
                if self.board[i][j] * sideSign == -ChessBoard.KING_CODE:
                    attackingPieces.append((i, j, ChessBoard.KING_CODE))

        for direction in rookMoveMap:

            i = kingLine + direction[0]
            j = kingColumn + direction[1]

            while (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if self.board[i][j] * sideSign == -ChessBoard.ROOK_CODE or \
                    self.board[i][j] * sideSign == -ChessBoard.QUEEN_CODE:

                    attackingPieces.append((i, j, abs(self.board[i][j])))
                    break
                elif self.board[i][j] != 0:
                    break

                i += direction[0]
                j += direction[1]

        for direction in bishopMoveMap:

            i = kingLine + direction[0]
            j = kingColumn + direction[1]

            while (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if self.board[i][j] * sideSign == -ChessBoard.BISHOP_CODE or \
                    self.board[i][j] * sideSign == -ChessBoard.QUEEN_CODE:

                    attackingPieces.append((i, j, abs(self.board[i][j])))
                    break
                elif self.board[i][j] != 0:
                    break

                i += direction[0]
                j += direction[1]

        i, j = kingLine - sideSign * boardOrientationSign, kingColumn

        if i >= 0 and i < 8:

            if j + 1 < 8:
                if self.board[i][j+1] * sideSign == -ChessBoard.PAWN_CODE:
                    attackingPieces.append((i, j+1, ChessBoard.PAWN_CODE))

            if j - 1 >= 0:
                if self.board[i][j-1] * sideSign == -ChessBoard.PAWN_CODE:
                    attackingPieces.append((i, j-1, ChessBoard.PAWN_CODE))

        return attackingPieces

    def _getBlockingPieces(self, blockLine: int, blockColumn: int, sideSign: int) -> List[Tuple[int, int, int]]:

        knightMoveMap = ((2, 1), (2, -1), (-2, 1), (-2, -1), (1, 2), (-1, 2), (1, -2), (-1, -2))
        rookMoveMap = ((1, 0), (0, 1), (-1, 0), (0, -1))
        bishopMoveMap = ((1, 1), (-1, 1), (-1, -1), (1, -1))
        kingMoveMap = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))

        boardOrientationSign = 1

        if self.boardOrientation == "reversed":
            boardOrientationSign = -1

        blockingPieces = []

        for direction in knightMoveMap:

            i = blockLine + direction[0]
            j = blockColumn + direction[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):
                if self.board[i][j] * sideSign == ChessBoard.KNIGHT_CODE:
                    blockingPieces.append((i, j, ChessBoard.KNIGHT_CODE))

        for direction in rookMoveMap:

            i = blockLine + direction[0]
            j = blockColumn + direction[1]

            while (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if self.board[i][j] * sideSign == ChessBoard.ROOK_CODE or \
                    self.board[i][j] * sideSign == ChessBoard.QUEEN_CODE:

                    blockingPieces.append((i, j, abs(self.board[i][j])))
                    break
                elif self.board[i][j] != 0:
                    break

                i += direction[0]
                j += direction[1]

        for direction in bishopMoveMap:

            i = blockLine + direction[0]
            j = blockColumn + direction[1]

            while (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if self.board[i][j] * sideSign == ChessBoard.BISHOP_CODE or \
                    self.board[i][j] * sideSign == ChessBoard.QUEEN_CODE:

                    blockingPieces.append((i, j, abs(self.board[i][j])))
                    break
                elif self.board[i][j] != 0:
                    break

                i += direction[0]
                j += direction[1]

        i, j = blockLine + sideSign * boardOrientationSign, blockColumn

        if i >= 0 and i < 8:
            if self.board[i][j] * sideSign == ChessBoard.PAWN_CODE:
                blockingPieces.append((i, j, ChessBoard.PAWN_CODE))
            else:
                i += sideSign * boardOrientationSign
                if (i >= 0 and i < 8) and \
                    self.board[i][j] * sideSign == ChessBoard.PAWN_CODE:
                    if (i == 1 and sideSign * boardOrientationSign == -1) or \
                        (i == 6 and sideSign * boardOrientationSign == 1):
                        blockingPieces.append((i, j, ChessBoard.PAWN_CODE))

        return blockingPieces
        

    def isCheckMate(self, side: str) -> bool:
        
        if not self.isCheck(side):
            return False

        sideSign = 1

        if side == "black":
            sideSign = -1

        kingLine = 0
        kingColumn = 0

        for i in range(8):
            for j in range(8):
                if self.board[i][j] * sideSign == ChessBoard.KING_CODE:
                    kingLine = i
                    kingColumn = j
                    break

        # Check if king can move out of check
        moveMap = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
        checkMateAvoided = False
    
        for direction in moveMap:

            i = kingLine + direction[0]
            j = kingColumn + direction[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if self.board[i][j] == 0:
                    
                    self.board[i][j] = self.board[kingLine][kingColumn]
                    self.board[kingLine][kingColumn] = 0

                    if not self.isCheck(side):
                        checkMateAvoided = True
        
                    self.board[kingLine][kingColumn] = self.board[i][j]
                    self.board[i][j] = 0

            if checkMateAvoided:
                return False

        kingAttackingPieces = self._getAttackingPieces(kingLine, kingColumn, sideSign)

        # If 2 or more pieces are attacking the king at the same time and the 
        # king can't move, then it's checkmate since you can't capture or
        # block 2 pieces at the same time
        if len(kingAttackingPieces) > 1:
            return True

        else:
            
            # Check if a piece can capture the attacking piece
            threatPiece = kingAttackingPieces[0]
            threatAttackingPieces = self._getAttackingPieces(threatPiece[0], threatPiece[1], -sideSign)
            for piece in threatAttackingPieces:

                # Check if the piece is actually pinned
                self.board[threatPiece[0]][threatPiece[1]] = piece[2] if sideSign == 1 else -piece[2]
                self.board[piece[0]][piece[1]] = 0
                
                isStillCheck = self.isCheck(side)

                self.board[piece[0]][piece[1]] = piece[2] if sideSign == 1 else -piece[2]
                self.board[threatPiece[0]][threatPiece[1]] = threatPiece[2] if sideSign == -1 else -threatPiece[2]

                if not isStillCheck:
                    return False

            # Check if a piece can block the attacking piece

            # Knights and pawns can't be blocked
            if threatPiece[2] == ChessBoard.KNIGHT_CODE or \
                threatPiece[2] == ChessBoard.PAWN_CODE:
                return True
            
            if threatPiece[2] == ChessBoard.ROOK_CODE or \
                threatPiece[2] == ChessBoard.QUEEN_CODE:

                if kingLine == threatPiece[0]:
                    minColumn = min(kingColumn, threatPiece[1])
                    maxColumn = max(kingColumn, threatPiece[1])
                    for column in range(minColumn+1, maxColumn):
                        
                        blockingPieces = self._getBlockingPieces(kingLine, column, sideSign)

                        for blockingPiece in blockingPieces:

                            # Check if the piece is actually pinned
                            self.board[kingLine][column] = blockingPiece[2] if sideSign == -1 else -blockingPiece[2]
                            self.board[blockingPiece[0]][blockingPiece[1]] = 0
                            
                            isStillCheck = self.isCheck(side)

                            self.board[blockingPiece[0]][blockingPiece[1]] = blockingPiece[2] if sideSign == -1 else -blockingPiece[2]
                            self.board[kingLine][column] = 0 

                            if not isStillCheck:
                                return False
                elif kingColumn == threatPiece[1]:
                    minLine = min(kingLine, threatPiece[0])
                    maxLine = max(kingLine, threatPiece[0])
                    for line in range(minLine+1, maxLine):
                        
                        blockingPieces = self._getBlockingPieces(line, kingColumn, sideSign)

                        for blockingPiece in blockingPieces:

                            # Check if the piece is actually pinned
                            self.board[line][kingColumn] = blockingPiece[2] if sideSign == -1 else -blockingPiece[2]
                            self.board[blockingPiece[0]][blockingPiece[1]] = 0
                            
                            isStillCheck = self.isCheck(side)

                            self.board[blockingPiece[0]][blockingPiece[1]] = blockingPiece[2] if sideSign == -1 else -blockingPiece[2]
                            self.board[line][kingColumn] = 0 

                            if not isStillCheck:
                                return False
            if threatPiece[2] == ChessBoard.BISHOP_CODE or \
                    threatPiece[2] == ChessBoard.QUEEN_CODE:

                minLine = min(kingLine, threatPiece[0])
                maxLine = max(kingLine, threatPiece[0])

                minColumn = min(kingColumn, threatPiece[1])
                maxColumn = max(kingColumn, threatPiece[1])

                # Piece is on the first diagonal
                if (kingLine - threatPiece[0]) * (kingColumn - threatPiece[1]) > 0:

                    for i in range(1, maxLine - minLine):
                        
                        blockingPieces = self._getBlockingPieces(minLine+i, minColumn+i, sideSign)

                        for blockingPiece in blockingPieces:

                            # Check if the piece is actually pinned
                            self.board[minLine+i][minColumn+i] = blockingPiece[2] if sideSign == 1 else -blockingPiece[2]
                            self.board[blockingPiece[0]][blockingPiece[1]] = 0
                            
                            isStillCheck = self.isCheck(side)

                            self.board[blockingPiece[0]][blockingPiece[1]] = blockingPiece[2] if sideSign == 1 else -blockingPiece[2]
                            self.board[minLine+i][minColumn+i] = 0 

                            if not isStillCheck:
                                return False
                elif (kingLine - threatPiece[0]) * (kingColumn - threatPiece[1]) < 0:
                    
                    for i in range(1, maxLine - minLine):
                        
                        blockingPieces = self._getBlockingPieces(minLine+i, maxColumn-i, sideSign)

                        for blockingPiece in blockingPieces:

                            # Check if the piece is actually pinned
                            self.board[minLine+i][maxColumn-i] = blockingPiece[2] if sideSign == 1 else -blockingPiece[2]
                            self.board[blockingPiece[0]][blockingPiece[1]] = 0
                            
                            isStillCheck = self.isCheck(side)

                            self.board[blockingPiece[0]][blockingPiece[1]] = blockingPiece[2] if sideSign == 1 else -blockingPiece[2]
                            self.board[minLine+i][maxColumn-i] = 0 

                            if not isStillCheck:
                                return False

        return True


    def checkChessBoardState(self):
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

                if cell == ChessBoard.PAWN_CODE:
                    print("p", end=' ')
                if cell == ChessBoard.BISHOP_CODE:
                    print("B", end=' ')
                if cell == ChessBoard.KNIGHT_CODE:
                    print("N", end=' ')
                if cell == ChessBoard.QUEEN_CODE:
                    print("Q", end=' ')
                if cell == ChessBoard.KING_CODE:
                    print("K", end=' ')
                if cell == ChessBoard.ROOK_CODE:
                    print("R", end=' ')
            print()

