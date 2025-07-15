from typing import List, Tuple
from copy import deepcopy

class ChessBoard:

    PAWN_CODE = 1
    KNIGHT_CODE = 2
    BISHOP_CODE = 3
    ROOK_CODE = 5
    QUEEN_CODE = 10
    KING_CODE = 100000000

    # boardOrientation = normal (white bottom, black top), reversed (black bottom, white top)
    def __init__(self, board=None, boardOrientation: str="normal"):

        self.whiteKingMoved = False
        self.blackKingMoved = False
        self.whiteKingSideRookMoved = False
        self.whiteQueenSideRookMoved = False
        self.blackKingSideRookMoved = False
        self.blackQueenSideRookMoved = False

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
                self.board[0][3] = ChessBoard.KING_CODE
                self.board[0][4] = ChessBoard.QUEEN_CODE
                self.board[-1][3] = -ChessBoard.KING_CODE
                self.board[-1][4] = -ChessBoard.QUEEN_CODE
        else:
            self.board = deepcopy(board)

    def getKingCoordinates(self, side: str) -> Tuple[int, int]:
        
        sideSign = 1

        if side == "black":
            sideSign = -1
    
        for i in range(8):
            for j in range(8):
                if self.board[i][j] * sideSign == ChessBoard.KING_CODE:
                    return i, j

        return -1, -1

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

        kingLine, kingColumn = self.getKingCoordinates(side)

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

    def isDraw(self, side: str) -> bool:

        if not self.isCheck(side):
            legalMoves = self.getLegalMoves(side)
            if len(legalMoves) == 0:
                return True

        return False

    def _getLegalKnightMoves(self, knightLine: int, knightColumn: int, sideSign: int) -> List[List[List[int]]]:
        
        knightMoves = []
        knightMoveMap = ((2, 1), (2, -1), (-2, 1), (-2, -1), (1, 2), (-1, 2), (1, -2), (-1, -2))

        for moveMap in knightMoveMap:

            i = knightLine + moveMap[0]
            j = knightColumn + moveMap[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if sideSign * self.board[i][j] <= 0:
                    newBoard = deepcopy(self.board)
                    newBoard[knightLine][knightColumn] = 0
                    newBoard[i][j] = sideSign * ChessBoard.KNIGHT_CODE

                    _ = ChessBoard(newBoard, self.boardOrientation)
                    if not _.isCheck("white" if sideSign == 1 else "black"):
                        knightMoves.append(newBoard)

        return knightMoves

    def _getLegalBishopMoves(self, bishopLine: int, bishopColumn: int, sideSign: int) -> List[List[List[int]]]:
        
        bishopMoves = []
        bishopMoveMap = ((1, 1), (-1, 1), (-1, -1), (1, -1))

        for moveMap in bishopMoveMap:

            i = bishopLine + moveMap[0]
            j = bishopColumn + moveMap[1]

            while (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if sideSign * self.board[i][j] <= 0:

                    newBoard = deepcopy(self.board)
                    newBoard[bishopLine][bishopColumn] = 0
                    newBoard[i][j] = sideSign * ChessBoard.BISHOP_CODE
                    _ = ChessBoard(newBoard, self.boardOrientation)

                    if not _.isCheck("white" if sideSign == 1 else "black"):
                        bishopMoves.append(newBoard)

                    if sideSign * self.board[i][j] < 0:
                        break
                else:
                    break

                i += moveMap[0]
                j += moveMap[1]
                    

        return bishopMoves

    def _getLegalQueenMoves(self, queenLine: int, queenColumn: int, sideSign: int) -> List[List[List[int]]]:
        
        queenMoves = []
        queenMoveMap = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))

        for moveMap in queenMoveMap:

            i = queenLine + moveMap[0]
            j = queenColumn + moveMap[1]

            while (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if sideSign * self.board[i][j] <= 0:

                    newBoard = deepcopy(self.board)
                    newBoard[queenLine][queenColumn] = 0
                    newBoard[i][j] = sideSign * ChessBoard.QUEEN_CODE
                    _ = ChessBoard(newBoard, self.boardOrientation)

                    if not _.isCheck("white" if sideSign == 1 else "black"):
                        queenMoves.append(newBoard)

                    if sideSign * self.board[i][j] < 0:
                        break
                else:
                    break

                i += moveMap[0]
                j += moveMap[1]
                    

        return queenMoves

    def _getLegalRookMoves(self, rookLine: int, rookColumn: int, sideSign: int) -> List[List[List[int]]]:
        
        rookMoves = []
        rookMoveMap = ((1, 0), (-1, 0), (0, 1), (0, -1))

        for moveMap in rookMoveMap:

            i = rookLine + moveMap[0]
            j = rookColumn + moveMap[1]

            while (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if sideSign * self.board[i][j] <= 0:
                    newBoard = deepcopy(self.board)
                    newBoard[rookLine][rookColumn] = 0
                    newBoard[i][j] = sideSign * ChessBoard.ROOK_CODE
                    _ = ChessBoard(newBoard, self.boardOrientation)

                    if not _.isCheck("white" if sideSign == 1 else "black"):
                        rookMoves.append(newBoard)

                    if sideSign * self.board[i][j] < 0:
                        break
                else:
                    break

                i += moveMap[0]
                j += moveMap[1]
                    

        return rookMoves

    def _getLegalPawnMoves(self, pawnLine: int, pawnColumn: int, sideSign: int, boardOrientationSign: int) -> List[List[List[int]]]:

        pawnMoves = []

        i = pawnLine - sideSign * boardOrientationSign 

        if i >= 0 and i < 8:

            for j in [pawnColumn-1, pawnColumn+1]:

                if (j >= 0 and j < 8) and self.board[i][j] * sideSign < 0:
                    newBoard = deepcopy(self.board)
                    newBoard[i][j] = sideSign * ChessBoard.PAWN_CODE
                    newBoard[pawnLine][pawnColumn] = 0
                    _ = ChessBoard(newBoard, "normal" if boardOrientationSign == 1 else "reversed")
                    if not _.isCheck("white" if sideSign == 1 else "black"):

                        if (i == 0 and boardOrientationSign * sideSign == 1) or \
                            (i == 7 and boardOrientationSign * sideSign == -1):

                            for promotionPieceCode in [ChessBoard.QUEEN_CODE, ChessBoard.KNIGHT_CODE, ChessBoard.BISHOP_CODE, ChessBoard.ROOK_CODE]:

                                newBoard[i][j] = promotionPieceCode * sideSign
                                pawnMoves.append(deepcopy(newBoard))
                        else:
                            pawnMoves.append(newBoard)

            j = pawnColumn

            if self.board[i][j] * sideSign == 0:
                newBoard = deepcopy(self.board)
                newBoard[i][j] = sideSign * ChessBoard.PAWN_CODE
                newBoard[pawnLine][pawnColumn] = 0
                _ = ChessBoard(newBoard, "normal" if boardOrientationSign == 1 else "reversed")
                if not _.isCheck("white" if sideSign == 1 else "black"):

                    if (i == 0 and boardOrientationSign * sideSign == 1) or \
                        (i == 7 and boardOrientationSign * sideSign == -1):

                        for promotionPieceCode in [ChessBoard.QUEEN_CODE, ChessBoard.KNIGHT_CODE, ChessBoard.BISHOP_CODE, ChessBoard.ROOK_CODE]:

                            newBoard[i][j] = promotionPieceCode * sideSign
                            pawnMoves.append(deepcopy(newBoard))
                    else:
                        pawnMoves.append(newBoard)

                # Check if pawn is on starting square
                if (pawnLine == 1 and boardOrientationSign * sideSign == -1) or \
                    (pawnLine == 6 and boardOrientationSign * sideSign == 1):

                    i -= sideSign * boardOrientationSign 

                    if self.board[i][j] * sideSign == 0:
                        newBoard = deepcopy(self.board)
                        newBoard[i][j] = sideSign * ChessBoard.PAWN_CODE
                        newBoard[pawnLine][pawnColumn] = 0
                        _ = ChessBoard(newBoard, "normal" if boardOrientationSign == 1 else "reversed")
                        if not _.isCheck("white" if sideSign == 1 else "black"):
                            pawnMoves.append(newBoard)


        return pawnMoves

    def _getLegalKingMoves(self, kingLine: int, kingColumn: int, sideSign: int) -> List[List[List[int]]]:
        
        kingMoveMap = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
        kingMoves = []

        for moveMap in kingMoveMap:

            i = kingLine + moveMap[0]
            j = kingColumn + moveMap[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):

                if self.board[i][j] <= 0:
                    newBoard = deepcopy(self.board)
                    newBoard[i][j] = sideSign * ChessBoard.KING_CODE
                    newBoard[kingLine][kingColumn] = 0
                    _ = ChessBoard(newBoard, self.boardOrientation)

                    if not _.isCheck("white" if sideSign == 1 else "black"):
                        kingMoves.append(newBoard)

        return kingMoves


    def getLegalCheckMoves(self, side: str):

        kingLine, kingColumn = self.getKingCoordinates(side)
        kingMoveMap = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))

        sideSign = 1

        if side == "black":
            sideSign = -1

        legalMoves = []

        for moveMap in kingMoveMap:

            i = kingLine + moveMap[0]
            j = kingLine + moveMap[1]

            if (i >= 0 and i < 8) and (j >= 0 and j < 8):
                if self.board[i][j] * sideSign <= 0:
                    newBoard = deepcopy(self.board)
                    newBoard[i][j] = sideSign * ChessBoard.KING_CODE
                    newBoard[kingLine][kingColumn] = 0
                    _ = ChessBoard(newBoard, self.boardOrientation)

                    if not _.isCheck(side):
                        legalMoves.append(newBoard)

        kingAttackingPieces = self._getAttackingPieces(kingLine, kingColumn, sideSign)

        # Check if a piece can capture the attacking piece
        threatPiece = kingAttackingPieces[0]
        threatAttackingPieces = self._getAttackingPieces(threatPiece[0], threatPiece[1], -sideSign)
        for piece in threatAttackingPieces:

            newBoard = deepcopy(self.board)
            newBoard[threatPiece[0]][threatPiece[1]] = piece[2] * sideSign
            newBoard[piece[0]][piece[1]] = 0

            _ = ChessBoard(newBoard, self.boardOrientation)

            if not _.isCheck(side):
                legalMoves.append(newBoard)

        # Check if a piece can block the attacking piece

        # Knights and pawns can't be blocked
        if threatPiece[2] == ChessBoard.KNIGHT_CODE or \
            threatPiece[2] == ChessBoard.PAWN_CODE:
            return legalMoves
        
        if threatPiece[2] == ChessBoard.ROOK_CODE or \
            threatPiece[2] == ChessBoard.QUEEN_CODE:

            if kingLine == threatPiece[0]:

                minColumn = min(kingColumn, threatPiece[1])
                maxColumn = max(kingColumn, threatPiece[1])

                for column in range(minColumn+1, maxColumn):
                    
                    blockingPieces = self._getBlockingPieces(kingLine, column, sideSign)

                    for blockingPiece in blockingPieces:

                        # Check if the piece is actually pinned
                        newBoard = deepcopy(self.board)
                        newBoard[kingLine][column] = blockingPiece[2] * (-sideSign)
                        newBoard[blockingPiece[0]][blockingPiece[1]] = 0
                        _ = ChessBoard(newBoard, self.boardOrientation)
                        
                        if not _.isCheck(side):
                            legalMoves.append(newBoard)

            elif kingColumn == threatPiece[1]:

                minLine = min(kingLine, threatPiece[0])
                maxLine = max(kingLine, threatPiece[0])

                for line in range(minLine+1, maxLine):
                    
                    blockingPieces = self._getBlockingPieces(line, kingColumn, sideSign)

                    for blockingPiece in blockingPieces:

                        newBoard = deepcopy(self.board)
                        newBoard[line][kingColumn] = blockingPiece[2] * (-sideSign)
                        newBoard[blockingPiece[0]][blockingPiece[1]] = 0
                        _ = ChessBoard(newBoard, self.boardOrientation)

                        if not _.isCheck(side):
                            legalMoves.append(newBoard)

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

                        newBoard = deepcopy(self.board)
                        newBoard[minLine+i][minColumn+i] = blockingPiece[2] * (sideSign)
                        newBoard[blockingPiece[0]][blockingPiece[1]] = 0
                        _ = ChessBoard(newBoard, self.boardOrientation)
                    
                        if not _.isCheck(side):
                            legalMoves.append(newBoard)

            elif (kingLine - threatPiece[0]) * (kingColumn - threatPiece[1]) < 0:
                
                for i in range(1, maxLine - minLine):
                    
                    blockingPieces = self._getBlockingPieces(minLine+i, maxColumn-i, sideSign)

                    for blockingPiece in blockingPieces:

                        newBoard = deepcopy(self.board)
                        newBoard[minLine+i][minColumn-i] = blockingPiece[2] * (sideSign)
                        newBoard[blockingPiece[0]][blockingPiece[1]] = 0
                        _ = ChessBoard(newBoard, self.boardOrientation)
                    
                        if not _.isCheck(side):
                            legalMoves.append(newBoard)

        return legalMoves


    def getLegalMoves(self, side: str) -> List[List[List[int]]]:

        sideSign = 1

        if side == "black":
            sideSign = -1

        boardOrientationSign = 1

        if self.boardOrientation == "reversed":
            boardOrientationSign = -1

        if self.isCheck(side):

            # If it's checkmate then there's no available moves left
            if self.isCheckMate(side):
                return []

            # If it's check but not checkmate, check moves which get the king out of check
            return self.getLegalCheckMoves(side)
        
        legalMoves = []

        if self.canCastle(side, "king"):
            _ = deepcopy(self.board)
            self.castle(side, "king")
            legalMoves.append(self.board)
            self.board = _

        if self.canCastle(side, "queen"):
            _ = deepcopy(self.board)
            self.castle(side, "queen")
            legalMoves.append(self.board)
            self.board = _

        for i in range(8):
            for j in range(8):
                
                if sideSign * self.board[i][j] > 0:

                    possibleMoves = []

                    if sideSign * self.board[i][j] == ChessBoard.PAWN_CODE:
                        possibleMoves = self._getLegalPawnMoves(i, j, sideSign, boardOrientationSign)
                    elif sideSign * self.board[i][j] == ChessBoard.KNIGHT_CODE:
                        possibleMoves = self._getLegalKnightMoves(i, j, sideSign)
                    elif sideSign * self.board[i][j] == ChessBoard.BISHOP_CODE:
                        possibleMoves = self._getLegalBishopMoves(i, j, sideSign)
                    elif sideSign * self.board[i][j] == ChessBoard.ROOK_CODE:
                        possibleMoves = self._getLegalRookMoves(i, j, sideSign)
                    elif sideSign * self.board[i][j] == ChessBoard.QUEEN_CODE:
                        possibleMoves = self._getLegalQueenMoves(i, j, sideSign)
                    elif sideSign * self.board[i][j] == ChessBoard.KING_CODE:
                        possibleMoves = self._getLegalKingMoves(i, j, sideSign)

                    legalMoves += possibleMoves

        return legalMoves

    def isMoveLegal(self, board: List[List[int]], side: str) -> bool:

        legalMoves = self.getLegalMoves(side)

        return (board in legalMoves)


    def checkChessBoardState(self):
        pass

    def getDifference(self, board: List[List[int]]) -> Tuple[int, int, int, int, int]:
        
        initialLine, initialColumn = -1, -1
        finalLine, finalColumn = -1, -1
        transformation = 0

        for i in range(8):
            for j in range(8):
                if board[i][j] != self.board[i][j]:
                    if board[i][j] == 0:
                        initialLine = i
                        initialColumn = j
                    else:
                        finalLine = i
                        finalColumn = j

        if abs(self.board[initialLine][initialColumn]) == ChessBoard.PAWN_CODE:
            sideSign = 1 if board[i][j] > 0 else -1
            boardOrientationSign = 1 if self.boardOrientation == "normal" else -1
            if finalLine == 0 and sideSign * boardOrientationSign == 1:
                transformation = board[finalLine][finalColumn]
            elif finalLine == 7 and sideSign * boardOrientationSign == -1:
                transformation = board[finalLine][finalColumn]

        return initialColumn, initialLine, finalColumn, finalLine, transformation

    def makeMove(self, initLine: int, initColumn: int, finalLine: int, finalColumn, transformation: int) -> None:

        if initLine == 0:
            if initColumn == 0:
                if self.boardOrientation == "normal":
                    self.blackQueenSideRookMoved = True
                else:
                    self.whiteKingSideRookMoved = True
            elif initColumn == 7:
                if self.boardOrientation == "normal":
                    self.blackKingSideRookMoved = True
                else:
                    self.whiteQueenSideRookMoved = True

        if initLine == 7:
            if initColumn == 0:
                if self.boardOrientation == "normal":
                    self.whiteQueenSideRookMoved = True
                else:
                    self.blackKingSideRookMoved = True
            elif initColumn == 7:
                if self.boardOrientation == "normal":
                    self.whiteKingSideRookMoved = True
                else:
                    self.blackQueenSideRookMoved = True

        self.board[finalLine][finalColumn] = self.board[initLine][initColumn]
        self.board[initLine][initColumn] = 0

        if abs(self.board[finalLine][finalColumn]) == ChessBoard.KING_CODE:
            if self.board[finalLine][finalColumn] < 0:
                self.blackKingMoved = True
            else:
                self.whiteKingMoved = True

        if transformation > 0 and abs(self.board[finalLine][finalColumn]) == ChessBoard.PAWN_CODE:
            self.board[finalLine][finalColumn] = transformation

    def canCastle(self, side: str, direction: str) -> bool:

        if self.isCheck(side):
            return False

        if side == "white":
            if self.whiteKingMoved:
                return False
            if direction == "king" and self.whiteKingSideRookMoved:
                return False
            if direction == "queen" and self.whiteQueenSideRookMoved:
                return False


        if side == "black":
            if self.blackKingMoved:
                return False
            if direction == "king" and self.blackKingSideRookMoved:
                return False
            if direction == "queen" and self.blackQueenSideRookMoved:
                return False

        if self.boardOrientation == "normal":
            if side == "white":
                if direction == "king":
                    if self.board[-1][4] == ChessBoard.KING_CODE and \
                        self.board[-1][5] == 0 and \
                        self.board[-1][6] == 0 and \
                        self.board[-1][7] == ChessBoard.ROOK_CODE:

                        for i in range(5, 7):
                            self.board[0][i] = ChessBoard.KING_CODE
                            self.board[0][i-1] = 0

                            if self.isCheck(side):
                                self.board[0][4] = ChessBoard.KING_CODE
                                self.board[0][i] = 0
                                return False

                        self.board[-1][4] = ChessBoard.KING_CODE
                        self.board[-1][6] = 0

                        return True

                elif direction == "queen":
                    if self.board[-1][4] == ChessBoard.KING_CODE and \
                        self.board[-1][3] == 0 and \
                        self.board[-1][2] == 0 and \
                        self.board[-1][1] == 0 and \
                        self.board[-1][0] == ChessBoard.ROOK_CODE:

                        for i in range(3, 1, -1):

                            self.board[-1][i] = ChessBoard.KING_CODE
                            self.board[-1][i+1] = 0

                            if self.isCheck(side):
                                self.board[-1][4] = ChessBoard.KING_CODE
                                self.board[-1][i] = 0
                                return False

                        self.board[-1][4] = ChessBoard.KING_CODE
                        self.board[-1][2] = 0

                        return True

            elif side == "black":
                if direction == "king":
                    if self.board[0][4] == -ChessBoard.KING_CODE and \
                        self.board[0][5] == 0 and \
                        self.board[0][6] == 0 and \
                        self.board[0][7] == -ChessBoard.ROOK_CODE:

                        for i in range(5, 7):
                            self.board[0][i] = -ChessBoard.KING_CODE
                            self.board[0][i-1] = 0

                            if self.isCheck(side):
                                self.board[0][4] = -ChessBoard.KING_CODE
                                self.board[0][i] = 0
                                return False

                        self.board[0][4] = -ChessBoard.KING_CODE
                        self.board[0][6] = 0

                        return True

                elif direction == "queen":
                    if self.board[0][4] == -ChessBoard.KING_CODE and \
                        self.board[0][3] == 0 and \
                        self.board[0][2] == 0 and \
                        self.board[0][1] == 0 and \
                        self.board[0][0] == -ChessBoard.ROOK_CODE:

                        for i in range(3, 1, -1):

                            self.board[0][i] = -ChessBoard.KING_CODE
                            self.board[0][i+1] = 0

                            if self.isCheck(side):
                                self.board[0][4] = -ChessBoard.KING_CODE
                                self.board[0][i] = 0
                                return False

                        self.board[0][4] = -ChessBoard.KING_CODE
                        self.board[0][2] = 0

                        return True

        else:
            if side == "black":
                if direction == "queen":
                    if self.board[-1][3] == -ChessBoard.KING_CODE and \
                        self.board[-1][4] == 0 and \
                        self.board[-1][5] == 0 and \
                        self.board[-1][6] == 0 and \
                        self.board[-1][7] == -ChessBoard.ROOK_CODE:

                        for i in range(4, 6):
                            self.board[-1][i] = -ChessBoard.KING_CODE
                            self.board[-1][i-1] = 0

                            if self.isCheck(side):
                                self.board[-1][3] = -ChessBoard.KING_CODE
                                self.board[-1][i] = 0
                                return False

                        self.board[-1][3] = -ChessBoard.KING_CODE
                        self.board[-1][5] = 0

                        return True

                if direction == "king":
                    if self.board[-1][3] == -ChessBoard.KING_CODE and \
                        self.board[-1][2] == 0 and \
                        self.board[-1][1] == 0 and \
                        self.board[-1][0] == -ChessBoard.ROOK_CODE:

                        for i in range(3, 0, -1):

                            self.board[-1][i] = -ChessBoard.KING_CODE
                            self.board[-1][i+1] = 0

                            if self.isCheck(side):
                                self.board[-1][3] = -ChessBoard.KING_CODE
                                self.board[-1][i] = 0
                                return False

                        self.board[-1][3] = -ChessBoard.KING_CODE
                        self.board[-1][1] = 0

                        return True

            elif side == "white":
                if direction == "queen":
                    if self.board[0][3] == ChessBoard.KING_CODE and \
                        self.board[0][4] == 0 and \
                        self.board[0][5] == 0 and \
                        self.board[0][6] == 0 and \
                        self.board[0][7] == ChessBoard.ROOK_CODE:

                        for i in range(4, 6):
                            self.board[0][i] = ChessBoard.KING_CODE
                            self.board[0][i-1] = 0

                            if self.isCheck(side):
                                self.board[0][3] = ChessBoard.KING_CODE
                                self.board[0][i] = 0
                                return False

                        self.board[0][3] = ChessBoard.KING_CODE
                        self.board[0][5] = 0

                        return True

                if direction == "king":
                    if self.board[0][3] == ChessBoard.KING_CODE and \
                        self.board[0][2] == 0 and \
                        self.board[0][1] == 0 and \
                        self.board[0][0] == ChessBoard.ROOK_CODE:

                        for i in range(3, 0, -1):

                            self.board[0][i] = ChessBoard.KING_CODE
                            self.board[0][i+1] = 0

                            if self.isCheck(side):
                                self.board[0][3] = ChessBoard.KING_CODE
                                self.board[0][i] = 0
                                return False
                        self.board[0][3] = ChessBoard.KING_CODE
                        self.board[0][1] = 0

                        return True

        return False

    def castle(self, side: str, direction: str) -> None:

        if self.boardOrientation == "normal":

            if side == "white" and direction == "king":
                self.board[-1][4] = 0
                self.board[-1][6] = ChessBoard.KING_CODE
                self.board[-1][5] = ChessBoard.ROOK_CODE
                self.board[-1][7] = 0
                self.whiteKingSideRookMoved = True
                self.whiteKingMoved = True

            elif side == "white" and direction == "queen":
                self.board[-1][4] = 0
                self.board[-1][2] = ChessBoard.KING_CODE
                self.board[-1][3] = ChessBoard.ROOK_CODE
                self.board[-1][0] = 0
                self.whiteQueenSideRookMoved = True
                self.whiteKingMoved = True

            elif side == "black" and direction == "king":
                self.board[0][4] = 0
                self.board[0][6] = ChessBoard.KING_CODE
                self.board[0][5] = ChessBoard.ROOK_CODE
                self.board[0][7] = 0
                self.blackQueenSideRookMoved = True
                self.blackKingMoved = True

            elif side == "black" and direction == "queen":
                self.board[0][4] = 0
                self.board[0][2] = ChessBoard.KING_CODE
                self.board[0][3] = ChessBoard.ROOK_CODE
                self.board[0][0] = 0
                self.blackKingSideRookMoved = True
                self.blackKingMoved = True

        else:

            if side == "black" and direction == "queen":
                self.board[-1][3] = 0
                self.board[-1][5] = ChessBoard.KING_CODE
                self.board[-1][4] = ChessBoard.ROOK_CODE
                self.board[-1][7] = 0
                self.blackQueenSideRookMoved = True
                self.blackKingMoved = True

            elif side == "black" and direction == "king":
                self.board[-1][3] = 0
                self.board[-1][1] = ChessBoard.KING_CODE
                self.board[-1][2] = ChessBoard.ROOK_CODE
                self.board[-1][0] = 0
                self.blackKingSideRookMoved = True
                self.blackKingMoved = True

            elif side == "white" and direction == "king":
                self.board[0][3] = 0
                self.board[0][1] = ChessBoard.KING_CODE
                self.board[0][2] = ChessBoard.ROOK_CODE
                self.board[0][0] = 0
                self.whiteKingSideRookMoved = True
                self.whiteKingMoved = True

            elif side == "white" and direction == "queen":
                self.board[0][3] = 0
                self.board[0][5] = ChessBoard.KING_CODE
                self.board[0][4] = ChessBoard.ROOK_CODE
                self.board[0][7] = 0
                self.whiteQueenSideRookMoved = True
                self.whiteKingMoved = True



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

