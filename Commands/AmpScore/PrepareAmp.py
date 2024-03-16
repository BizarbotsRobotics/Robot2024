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


class PrepareAmp(commands2.Command):
    """A command that will score a note into the Amp."""

    def __init__(self, shoober: Shoober) -> None:
        self.shoober = shoober
        super().__init__()
        self.addRequirements(self.shoober)
        

    def initialize(self):
        # self.shoober.setShooterMotorRPM(-3000)
        # self.shoober.setPivotPosition(120)
        self.run = True
        self.timer = 0
        self.timerTwo = 0

        self.shoober.setPivotPosition(140)

    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        pass
       
        
        


    def end(self, interrupted: bool):
       pass

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.shoober.getPivotAngle() > 139