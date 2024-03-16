import commands2
import commands2.cmd
import wpilib
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Vision.Vision import Vision


class DriveSpeakerCmd(commands2.Command):
    """A command that will align the robot to an April Tag. Not great for speed but last resort"""

    def __init__(self, swerveDrive : SwerveDrive, vision: Vision, x, y, rot) -> None:
        self.swerve = swerveDrive
        self.vision = vision
        self.x = x
        self.y = y
        self.rot = rot
        super().__init__()
        self.addRequirements(self.swerve, self.vision)

    def initialize(self):  
        self.steerAdjust = 0


    def execute(self):
        if self.vision.seeTarget():
            self.steerAdjust = .3
        else:
            self.steerAdjust = -self.swerve.headingPID.calculate(self.vision.getSpeakerCoords()[0], 0)
       
        self.swerve.driveFR(self.x(),self.y(),self.steerAdjust, False, True)

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False