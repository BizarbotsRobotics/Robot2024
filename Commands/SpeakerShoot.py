#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from Commands.AdjustNoteCmd import AdjustNoteCmd
from Commands.AdjustNoteShootCmd import AdjustNoteShootCmd
from Commands.HoldNote import HoldNote
from Commands.IntakeCmd import IntakeCmd
from Commands.Shoot import Shoot
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

import constants

class SpeakerShoot(commands2.SequentialCommandGroup):

    def __init__(self, shoober: Shoober):
        super().__init__(
        )
        self.addCommands(AdjustNoteShootCmd(shoober), commands2.WaitCommand(.05),Shoot(shoober))