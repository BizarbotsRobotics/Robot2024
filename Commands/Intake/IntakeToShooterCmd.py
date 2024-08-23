import commands2
from Commands.Intake.HoldNoteCmd import HoldNoteCmd
from Commands.Intake.IntakeCmd import IntakeCmd
from Commands.Shoot.AdjustNoteCmd import AdjustNoteCmd
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

import constants

class IntakeToShooterCmd(commands2.SequentialCommandGroup):
    """ Command that moves the note from the intake to the shooter"""
    def __init__(self, intake: Intake, conveyor: Conveyor, shoober: Shoober):
        super().__init__(
        )
        self.addCommands(IntakeCmd(conveyor, intake), HoldNoteCmd(shoober, conveyor), AdjustNoteCmd(shoober))