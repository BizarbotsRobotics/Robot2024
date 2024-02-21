#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
# import commands2.cmd
import wpimath.controller
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Shoober.Shoober import Shoober

# from Subsystems import SwerveDrive

# import constants


class HoldNote(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shoober:Shoober, conveyor: Conveyor) -> None:
        self.shoober = shoober
        self.conveyor = conveyor
        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        self.run = True
        super().__init__()
        self.addRequirements(self.shoober, self.conveyor)

    def initialize(self):
        """The initial subroutine of a command. Called once when the command is initially scheduled."""
        self.shoober.setPivotPosition(0)
    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        if self.shoober.getPivotAngle() < 5:
            self.shoober.setDualIndexerPower(-.4)
            self.conveyor.setConveyorPower(.8)

    def end(self, interrupted: bool):
        self.shoober.setDualIndexerPower(0)
        self.conveyor.setConveyorPower(0)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.shoober.getNoteStored()