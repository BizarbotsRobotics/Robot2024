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
import constants


class AmpScore(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shoober: Shoober) -> None:
        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        self.shoober = shoober
        super().__init__()
        self.addRequirements(self.shoober)
        self.timer = 0
        self.timerTwo = 0

    def initialize(self):
        # self.shoober.setShooterMotorRPM(-3000)
        self.shoober.setPivotPosition(120)
        self.shoober.setDualIndexerPosition(-.4)

    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        if  self.shoober.getPivotAngle() > 119:
            self.shoober.setBottomIndexerPower(-.7)
            self.shoober.setTopIndexerPower(.7)
            self.timer += 1
        if self.timerTwo > 30 and self.shoober.getPivotAngle() < 119:
            self.shoober.setDualIndexerPower(0)
        self.timerTwo+=1

        


    def end(self, interrupted: bool):
        self.shoober.setDualIndexerPower(0)
        self.shoober.setPivotPower(0)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        if self.timer > 60:
            return not self.shoober.getNoteStored()
        return False