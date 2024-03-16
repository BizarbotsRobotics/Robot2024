import commands2
from wpilib import DriverStation
from Commands.Autos.TwoNote.ResetCmd import ResetCmd
from Commands.Intake.IntakeToShooterCmd import IntakeToShooterCmd
from Commands.Shoot.ShootCloseCmd import ShootCloseCmd
from Commands.Shoot.ShootManualCmd import ShootManualCmd
from Commands.StartConfig.StartConfigCmd import StartConfigCmd
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from pathplannerlib import path, auto

class OneNoteDriveCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )
        self.addCommands(StartConfigCmd(intake, shoober).withTimeout(2),ResetCmd(drive), ShootCloseCmd(shoober), commands2.waitcommand.WaitCommand(.5),
                                commands2.WaitCommand(4),auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("Drive")))
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed