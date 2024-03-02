#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import commands2.cmd
import wpimath.controller
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Vision.Vision import Vision
import constants


class SetShooterPivotCmd(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shoober: Shoober, vision: Vision) -> None:
        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        self.shoober = shoober
        self.vision = vision
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        self.shoober.setPivotPosition(self.vision.getDesiredPivot())

    def execute(self):
        self.shoober.setPivotPosition(self.vision.getDesiredPivot())
        """The main body of a command. Called repeatedly while the command is scheduled."""
        pass
        
    def end(self, interrupted: bool):
       pass

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  (self.vision.getDesiredPivot() -2) < self.shoober.getPivotAngle() < (self.vision.getDesiredPivot() + 2)