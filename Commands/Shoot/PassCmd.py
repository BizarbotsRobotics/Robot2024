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


class PassCmd(commands2.Command):
    """A command that will turn the robot to the specified angle."""

    def __init__(self, shoober: Shoober, vision: Vision) -> None:
        self.vision = vision
        self.shoober = shoober
        super().__init__()
        self.counter = 0
        self.timer = 0
        self.addRequirements(self.shoober)

    def initialize(self):
        self.counter = 0
        self.timer = 0
        self.shoober.setShooterMotorPower(-1)
        self.shoober.setPivotPositionLowPower(30)
        # self.shoober.setPivotPosition(0)

    def execute(self):
        if self.shoober.getShooterRPM() > -10:
            self.counter = self.counter + 1
        if self.counter > 5:
            self.shoober.resetPosition()
            self.shoober.setDualIndexerPosition(1)
            self.counter = 0
        if self.shoober.getPivotAngle() > 25:
            self.timer += 1
        if  self.shoober.getShooterRPM() < -3000 and self.shoober.getPivotAngle() > 25 and self.timer > 2:
            self.shoober.setDualIndexerPower(-1)
        
    def end(self, interrupted: bool):
        self.shoober.setShooterMotorPower(0)
        self.shoober.setDualIndexerPower(0)
        self.shoober.setPivotPosition(1)



    def isFinished(self) -> bool:
        return  not self.shoober.getNoteStored()