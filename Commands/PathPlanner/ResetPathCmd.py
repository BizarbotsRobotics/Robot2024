import math
import commands2
import commands2.cmd
import wpilib
from wpimath import geometry
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Vision.Vision import Vision
from pathplannerlib import path, auto


class ResetPathCmd(commands2.Command):
    """A command that will run a path from pathplanner"""

    def __init__(self, swerveDrive : SwerveDrive, pathStr: str) -> None:
        self.swerve = swerveDrive
        self.cmd = None

        self.path = path.PathPlannerPath.fromPathFile(pathStr)
        super().__init__()
        self.addRequirements(self.swerve)

    def initialize(self):  
        self.counter = 0
        position = self.path.getPoint(0).position
        rotation = self.path.getPreviewStartingHolonomicPose().rotation()
        self.swerve.resetOdometry(geometry.Pose2d(geometry.Translation2d(position.X().real, position.Y().real), rotation))

    def execute(self):
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return True