#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import commands2.cmd
import wpimath.controller

from Subsystems import ShooterClimber

import constants


class TestShooterPivot(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shooter: ShooterClimber) -> None:
        self.shooter = shooter
        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        super().__init__()

    def initialize(self):
        """The initial subroutine of a command. Called once when the command is initially scheduled."""
        self.shooter.setIndexerPosition(10)

    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        pass
    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.getController().atSetpoint()