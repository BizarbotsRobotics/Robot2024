import commands2
from wpilib import DriverStation
from Commands.Autos.TwoNote.ResetCmd import ResetCmd
from Commands.Intake.IntakeCmd import IntakeCmd
from Commands.Shoot.ShootCloseCmd import ShootCloseCmd
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from pathplannerlib import path, auto

class TwoNoteAutoCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )
        self.addCommands(ResetCmd(drive), ShootCloseCmd(shoober), 
                         IntakeCmd(conveyor, intake).alongWith(
                                auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("TwoNoteBluePath")),
                                ), 
                             ShootCloseCmd(shoober) )
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed