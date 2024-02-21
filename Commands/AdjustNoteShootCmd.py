#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import ntcore
import wpilib
import commands2
# import commands2.cmd
import wpimath.controller
from Subsystems.Shoober.Shoober import Shoober

# from Subsystems import SwerveDrive

# import constants


class AdjustNoteShootCmd(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shoober:Shoober) -> None:
        self.shoober = shoober

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startServer()
        self.sd = self.inst.getTable("SmartDashboard")

        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        """The initial subroutine of a command. Called once when the command is initially scheduled."""
        self.shoober.resetPosition()
        self.runCommand = False
        
    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        self.shoober.setDualIndexerPosition(5)

    def end(self, interrupted: bool):
        self.shoober.setDualIndexerPower(0)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.shoober.getIndexerPosition() > 4