# rcj_soccer_player controller - ROBOT Y2

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):

    def run(self):
        self.right_motor.setVelocity(0)
        self.left_motor.setVelocity(0)