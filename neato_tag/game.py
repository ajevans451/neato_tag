
import numpy as np

#Color Calibrations
LOWER_PINK = np.array([116,0,69])
UPPER_PINK = np.array([255,73,150])
LOWER_BLUE = np.array([0,52,80])
UPPER_BLUE = np.array([26,104,255])
LOWER_YELLOW = np.array([123,137,0])
UPPER_YELLOW = np.array([194,203,69])
LOWER_GREEN = np.array([0,130,63])
UPPER_GREEN = np.array([76,255,109])
LOWER_PURPLE = np.array([74, 0, 115])
UPPER_PURPLE = np.array([109, 79, 184])
# CALIBRATION_MATRIX = np.array([[584.9932,0,0], [0, 584.6622,0], [377.3949,225.2839,1]])
CALIBRATION_MATRIX = np.array([[584.9932,0,377.3949], [0, 584.6622,225.2839], [0,0,1]])
FOCAL_Y = CALIBRATION_MATRIX[1, 1]
NOTE_HEIGHT = 0.0762
NUM_PIXELS=11943936


COLOR_TO_MASK = {
    "PINK":(LOWER_PINK, UPPER_PINK),
    "BLUE":(LOWER_BLUE, UPPER_BLUE),
    "YELLOW":(LOWER_YELLOW,UPPER_YELLOW),
    "GREEN":(LOWER_GREEN,UPPER_GREEN),
    "PURPLE": (LOWER_PURPLE, UPPER_PURPLE),
    }


class NeatoTag:
    """
    This class defines the parameters of the game of tag.
    It stores the number and colors of the players in the game.
    """

    def __init__(self, player_info):
        self.num_players = len(player_info)
        self.player_colors = [info[0] for info in player_info]
        self.hosts = [info[1] for info in player_info]


    def __repr__(self):
        return f"(Number of players: {self.num_players}, Active colors: {self.player_colors})"
    
    
NEATO_TAG=NeatoTag([
    ('PINK', '192.168.16.93'),
    ('PURPLE', '192.168.16.96'),
    ('GREEN', '192.168.17.207'),
])
