#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import commands2.cmd
import wpimath.controller
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober
import constants


class StartConfig(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shoober: Shoober, intake: Intake) -> None:
        """
        Turns to robot to the specified angle.

        :param: targetAngleDegrees The angle to turn to
        :param: drive The drive subsystem to
        """
        self.shoober = shoober
        super().__init__()
        self.addRequirements(self.shoober)
        self.intake = intake

    def initialize(self):
        self.run = True
        self.intake.resetPivotEncoder()
        self.shoober.setPivotPosition(130)
            
    def end(self, interrupted: bool):
        self.intake.setPivotPower(0)
        self.shoober.setPivotPower(0)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.shoober.getPivotAngle() < 5
        #self.shoober.setPivotPosition(130)

    def execute(self):
        if  self.shoober.getPivotAngle() > 129 and self.run:
            self.intake.setPivotPosition(2)
            self.run = False
        if self.run == False:
            self.shoober.setPivotPosition(0)