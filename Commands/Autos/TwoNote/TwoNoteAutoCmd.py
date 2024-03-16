import commands2
from wpilib import DriverStation
from Commands.Autos.TwoNote.Reset180Cmd import Reset180Cmd
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

class TwoNoteAutoCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )
        self.addCommands(StartConfigCmd(intake, shoober).withTimeout(2),ResetCmd(drive), ShootCloseCmd(shoober), commands2.waitcommand.WaitCommand(.5),
                         IntakeToShooterCmd(intake, conveyor, shoober).alongWith(
                                auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("TwoNoteBluePath")),
                                ).withTimeout(3), Reset180Cmd(drive),
                                auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("TwoNoteBluePathScore")),
                             ShootCloseCmd(shoober) )

        # self.addCommands(ResetCmd(drive),auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("TwoNoteBluePath")), Reset180Cmd(drive),auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("TwoNoteBluePathScore")))
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed