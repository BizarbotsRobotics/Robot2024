import commands2
from wpilib import DriverStation
from Commands.Autos.TwoNote.TwoNoteAutoCmd import TwoNoteAutoCmd
from Commands.Intake.IntakeToShooterCmd import IntakeToShooterCmd
from Commands.PathPlanner.ResetPathCmd import ResetPathCmd
from Commands.PathPlanner.RunPathCmd import RunPathCmd
from Commands.Shoot.ShootCloseCmd import ShootCloseCmd
from Commands.Shoot.ShootManualCmd import ShootManualCmd
from Commands.StartConfig.StartConfigCmd import StartConfigCmd
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from pathplannerlib import path, auto

class ThreeNoteAutoAltCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )
        self.addCommands(TwoNoteAutoCmd(drive, shoober, intake, conveyor),IntakeToShooterCmd(intake, conveyor, shoober).alongWith(
                                RunPathCmd(drive, "FourNoteBluePath"),
                                ).withTimeout(2.25), 
                                RunPathCmd(drive, "FourNoteBluePathScore").withTimeout(2.25),
                             ShootCloseCmd(shoober))
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed