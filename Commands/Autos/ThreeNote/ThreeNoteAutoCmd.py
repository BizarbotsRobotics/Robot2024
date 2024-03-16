import commands2
from wpilib import DriverStation
from Commands.Autos.TwoNote.Reset180Cmd import Reset180Cmd
from Commands.Autos.TwoNote.ResetCmd import ResetCmd
from Commands.Autos.TwoNote.TwoNoteAutoCmd import TwoNoteAutoCmd
from Commands.Intake.IntakeToShooterCmd import IntakeToShooterCmd
from Commands.Shoot.ShootCloseCmd import ShootCloseCmd
from Commands.Shoot.ShootManualCmd import ShootManualCmd
from Commands.StartConfig.StartConfigCmd import StartConfigCmd
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from pathplannerlib import path, auto

class ThreeNoteAutoCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )
        self.addCommands(TwoNoteAutoCmd(drive, shoober, intake, conveyor),ResetCmd(drive),IntakeToShooterCmd(intake, conveyor, shoober).alongWith(
                                auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("ThreeNoteBluePath")),
                                ).withTimeout(3), 
                                auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("ThreeNoteBluePathScore")),
                             ShootCloseCmd(shoober))
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed