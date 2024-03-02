import commands2
from Commands.StartConfig.IntakeStartPositionCmd import IntakeStartPositionCmd
from Commands.StartConfig.ShooterHoldPositionCmd import ShooterHoldPositionCmd
from Commands.StartConfig.ShooterStartPositionCmd import ShooterStartPositionCmd
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

import constants

class StartConfigCmd(commands2.SequentialCommandGroup):
    """ Command that moves the note from the intake to the shooter"""
    def __init__(self, intake: Intake, shoober: Shoober):
        super().__init__()
        self.addCommands(ShooterHoldPositionCmd(shoober), IntakeStartPositionCmd(intake), ShooterStartPositionCmd(shoober))