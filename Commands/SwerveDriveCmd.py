#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import commands2.cmd
import wpimath.controller
from Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
import constants


class SwerveDriveCmd(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, swerveDrive : SwerveDrive) -> None:
        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        self.swerveDrive = SwerveDrive
        super().__init__()
        self.addRequirements(self.swerveDrive)

    def initialize(self):
        self.shoober.setShooterMotorRPM(-5000)
        self.shoober.setPivotPosition(-10)

    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        if  self.shoober.getShooterRPM() < -4800 and self.shoober.getShooterRPM() > -5010 and self.shoober.getPivotAngle() < -9:
            self.shoober.setDualIndexerPower(-1)

        


    def end(self, interrupted: bool):
        self.shoober.setShooterMotorPower(0)
        self.shoober.setDualIndexerPower(0)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return not self.shoober.getNoteStored()