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


class ShootTrapCmd(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shoober: Shoober) -> None:
        """ """
        self.shoober = shoober

        super().__init__()
        self.addRequirements(self.shoober)


    def initialize(self):
        self.counter = 0
        self.timer = 0
        self.shoober.setShooterMotorRPM(-1500)
        self.shoober.setPivotPosition(0)

    def execute(self):
        if  self.shoober.getShooterRPM() < (-1400) and self.shoober.getPivotAngle() < 5:
            self.shoober.setDualIndexerPower(-1)
            self.timer += 1
        
    def end(self, interrupted: bool):
        self.shoober.setShooterMotorPower(0)
        self.shoober.setDualIndexerPower(0)
        self.shoober.setPivotPosition(1)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  not self.shoober.getNoteStored() and self.timer > 20
    