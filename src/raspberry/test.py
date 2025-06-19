from .game import Game
from .players.botPlayer import BotPlayer
from .players.humanPlayer import HumanPlayer

game = Game(HumanPlayer("white"), BotPlayer("black"))
game.printBoard()
game.playTurn()
print("-"*20)
game.printBoard()
