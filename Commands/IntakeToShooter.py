#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from Commands.AdjustNoteCmd import AdjustNoteCmd
from Commands.HoldNote import HoldNote
from Commands.IntakeCmd import IntakeCmd
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

import constants

class IntakeToShooter(commands2.SequentialCommandGroup):

    def __init__(self, intake: Intake, conveyor: Conveyor, shoober: Shoober):
        super().__init__(
        )
        self.addCommands(IntakeCmd(conveyor, intake), HoldNote(shoober, conveyor), commands2.WaitCommand(.1), AdjustNoteCmd(shoober))