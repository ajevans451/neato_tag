
import numpy as np
from typing import List, Tuple

#Color Calibrations
LOWER_PINK = np.array([155, 134, 30])
UPPER_PINK = np.array([168, 255, 255])
LOWER_GREEN = np.array([65, 120, 30])
UPPER_GREEN = np.array([80, 255, 255])
LOWER_PURPLE = np.array([118, 77, 53])
UPPER_PURPLE = np.array([141, 193, 255])

# Camera instrinsics
CALIBRATION_MATRIX = np.array([[584.9932,0,377.3949], [0, 584.6622,225.2839], [0,0,1]])
FOCAL_Y = CALIBRATION_MATRIX[1, 1]

# Height of a sticky note (meters)
NOTE_HEIGHT = 0.0762

# Color masks for each color
COLOR_TO_MASK = {
    "PINK":(LOWER_PINK, UPPER_PINK),
    "GREEN":(LOWER_GREEN,UPPER_GREEN),
    "PURPLE": (LOWER_PURPLE, UPPER_PURPLE),
}


class NeatoTag:
    """
    This class defines the parameters of the game of tag.
    It stores the number and colors of the players in the game.

    Attributes:
        num_players: an int, the number of neatos in this game
        player_colors: a list of strings, the names of each color of neato
        hosts: a list of strings, the IP Address for each neato playing
    """
    def __init__(self, player_info: List[Tuple[str, str]]):
        """
        Create a new instance of the tag game

        Args:
            player_info: a list of tuples giving the color and IP address for each neato playing
        """
        self.num_players = len(player_info)
        self.player_colors = [info[0] for info in player_info]
        self.hosts = [info[1] for info in player_info]

    def __repr__(self):
        return f"(Number of players: {self.num_players}, Active colors: {self.player_colors})"
    
    
NEATO_TAG = NeatoTag([
    ('GREEN', '192.168.17.207'),
    ('PURPLE', '192.168.16.86'),
    ('PINK', '192.168.16.93'),
])
