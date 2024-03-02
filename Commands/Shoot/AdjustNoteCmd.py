#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import ntcore
import wpilib
import commands2
import wpimath.controller
from Subsystems.Shoober.Shoober import Shoober


class AdjustNoteCmd(commands2.Command):
    """Adjusts note to be primed for shot"""

    def __init__(self, shoober:Shoober) -> None:
        self.shoober = shoober

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startServer()
        self.sd = self.inst.getTable("SmartDashboard")
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        self.shoober.resetPosition()
        self.runCommand = False
        
    def execute(self):
        if -.1 < self.shoober.getIndexerPosition() < .1:
            self.shoober.setDualIndexerPosition(-3)
            self.runCommand = True

    def end(self, interrupted: bool):
        self.shoober.setDualIndexerPosition(4)

    def isFinished(self) -> bool:
        return self.shoober.getIndexerPosition() < -2 and self.runCommand