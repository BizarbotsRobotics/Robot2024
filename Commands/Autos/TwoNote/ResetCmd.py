import math
import commands2
import commands2.cmd
from wpilib import DriverStation
from Subsystems.Drive.SwerveDrive import SwerveDrive
from wpimath import geometry

class ResetCmd(commands2.Command):
    """A command that will toggle the piston lock."""

    def __init__(self, drive: SwerveDrive) -> None:
        self.drive = drive
        super().__init__()
        self.addRequirements(self.drive)

    def initialize(self):
        pass
        self.drive.imu.reset()
        self.drive.resetOdometry(geometry.Pose2d(geometry.Translation2d(.71, 4.4), geometry.Rotation2d(-2.61)))

    def execute(self):
        pass
        
    def end(self, interrupted: bool):
       pass

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  True