import commands2
from wpilib import DriverStation
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

class TwoNoteAutoCmd(commands2.SequentialCommandGroup):

    def __init__(self, drive: SwerveDrive, shoober: Shoober, intake: Intake, conveyor: Conveyor):
        super().__init__(
        )
        self.addCommands(StartConfigCmd(intake, shoober).withTimeout(2), ShootCloseCmd(shoober), commands2.waitcommand.WaitCommand(.5),
                         IntakeToShooterCmd(intake, conveyor, shoober).deadlineWith(
                                RunPathCmd(drive, "TwoNoteBluePath")).withTimeout(2.2),
                                RunPathCmd(drive, "TwoNoteBluePathScore").withTimeout(2.2),

                             ShootCloseCmd(shoober))

        # self.addCommands(ResetCmd(drive),auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathyFile("TwoNoteBluePath")), Reset180Cmd(drive),auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("TwoNoteBluePathScore")))
        
    def field(self):
        return DriverStation.getAlliance() is DriverStation.Alliance.kRed