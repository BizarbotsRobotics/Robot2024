import commands2
from wpilib import DriverStation
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

class ThreeNoteRightAutoCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )
        self.addCommands(TwoNoteAutoCmd(drive, shoober, intake, conveyor),IntakeToShooterCmd(intake, conveyor, shoober).alongWith(
                                auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("ThreeNoteRightSide")),
                                ).withTimeout(3), 
                                auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("ThreeNoteRightSideScore")),
                            ShootCloseCmd(shoober))
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed