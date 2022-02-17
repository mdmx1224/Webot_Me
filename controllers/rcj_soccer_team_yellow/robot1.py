import math
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):

    def run(self):
        self.right_motor.setVelocity(0)
        self.left_motor.setVelocity(0)