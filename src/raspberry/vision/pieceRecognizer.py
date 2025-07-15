import os
import torch
import cv2
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
from .boardVision import BoardVisionModule
from ..chessBoard import ChessBoard

class PieceRecognizer:

    def __init__(self):
        NUM_CLASSES = 13
        self.DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model = models.mobilenet_v2(pretrained=False)
        self.model.classifier[1] = torch.nn.Linear(self.model.classifier[1].in_features, NUM_CLASSES)
        #self.model.fc = nn.Linear(self.model.fc.in_features, NUM_CLASSES)
        self.model.load_state_dict(torch.load("raspberry/vision/mobilenetv2_chess_piece_30.pt", map_location=self.DEVICE))
        self.model = self.model.to(self.DEVICE)
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406],
                                 [0.229, 0.224, 0.225])
        ])

        self.pieceCodes = [-ChessBoard.BISHOP_CODE, -ChessBoard.KING_CODE, -ChessBoard.KNIGHT_CODE, -ChessBoard.PAWN_CODE, -ChessBoard.QUEEN_CODE, -ChessBoard.ROOK_CODE, 0, ChessBoard.BISHOP_CODE, ChessBoard.KING_CODE, ChessBoard.KNIGHT_CODE, ChessBoard.PAWN_CODE, ChessBoard.QUEEN_CODE, ChessBoard.ROOK_CODE]

    def predictFrame(self, warpedImage):

        vision = BoardVisionModule()
        predictionImages = vision.getPieces(warpedImage)
        predictions = []
        for i in range(8):
            linePrediction = []
            for j in range(8):
                predictionImage = predictionImages[i][j]
                tensor = self.transform(Image.fromarray(predictionImage)).unsqueeze(0).to(self.DEVICE)
                with torch.no_grad():
                    output = self.model(tensor)
                    confidences = torch.softmax(output, dim=1).cpu().numpy()[0]
                    linePrediction.append(confidences)
                    _, pred = torch.max(output, 1)
            predictions.append(linePrediction)

        return predictions

    def getMaxPrediction(self, prediction) -> int:
        pred = np.argmax(prediction)
        return self.pieceCodes[pred]

    def getCodeOfPrediction(self, index: int) -> int:
        return self.pieceCodes[index]


