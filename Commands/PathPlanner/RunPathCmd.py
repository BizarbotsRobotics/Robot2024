import math
import commands2
import commands2.cmd
import wpilib
from wpimath import geometry
from Commands.PathPlanner.ResetPathCmd import ResetPathCmd
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Vision.Vision import Vision
from pathplannerlib import path, auto


class RunPathCmd(commands2.SequentialCommandGroup):
    """ Command that moves the note from the intake to the shooter"""
    def __init__(self, swerve: SwerveDrive, pathStr: str):
        super().__init__(
        )
        self.path = path.PathPlannerPath.fromPathFile(pathStr)
        self.addCommands(ResetPathCmd(swerve, pathStr), auto.AutoBuilder.followPath(self.path))