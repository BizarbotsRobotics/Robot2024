import math
import commands2
from wpilib import DriverStation
from wpimath import geometry
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

class OneNoteGTFOCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )  
        drive.imu.reset()
        drive.resetOdometry(geometry.Pose2d(geometry.Translation2d(0.75, 4.41), geometry.Rotation2d(math.radians(120))))
        self.addCommands(StartConfigCmd(intake, shoober).withTimeout(3),ResetCmd(drive), ShootCloseCmd(shoober), commands2.waitcommand.WaitCommand(.5),
                                commands2.WaitCommand(8),auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("OneNoteGTFO")))
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed