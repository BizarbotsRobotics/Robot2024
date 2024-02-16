#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import commands2.cmd
import wpimath.controller
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober
import constants


class IntakeCmd(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, conveyor:Conveyor, intake:Intake) -> None:
        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        self.conveyor = conveyor 
        self.intake = intake
        super().__init__()
        self.addRequirements(self.conveyor, self.intake)

    def initialize(self):
        self.conveyor.setConveyorPower(.7)
        self.intake.setIntakePower(1)

    def execute(self):        
        pass

    def end(self, interrupted: bool):
        self.conveyor.setConveyorPower(0)
        self.intake.setIntakePower(0)
        

    def isFinished(self) -> bool:
        return False