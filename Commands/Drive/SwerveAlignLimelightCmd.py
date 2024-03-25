import commands2
import commands2.cmd
import wpilib
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Vision.Vision import Vision


class SwerveAlignLimelightCmd(commands2.Command):
    """A command that will align the robot to an April Tag. Not great for speed but last resort"""

    def __init__(self, swerveDrive : SwerveDrive, vision: Vision) -> None:
        self.swerve = swerveDrive
        self.vision = vision
        super().__init__()
        self.addRequirements(self.swerve)
        self.pidControl = wpilib

    def initialize(self):  
        pass

    def execute(self):
        rot_limelight = self.swerve.limelight_aim_proportional()
        rot = rot_limelight

        forward_limelight = self.swerve.limelight_range_proportional()
        xSpeed = forward_limelight

        self.swerve.driveFR(0, 0, -rot, False, True);

    def end(self, interrupted: bool):
        self.swerve.driveFR(0,0,0, False, True)
        pass

    def isFinished(self) -> bool:
        return -1 < self.vision.getSpeakerCoords()[0] < 1