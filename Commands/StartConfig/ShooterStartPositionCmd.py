import wpilib
import commands2
import commands2.cmd
import wpimath.controller
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober
import constants


class ShooterStartPositionCmd(commands2.Command):
    """A command that will take the robot out of start config."""

    def __init__(self, shoober: Shoober) -> None:
        self.shoober = shoober
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        self.shoober.setPivotPosition(1)

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.shoober.setPivotPower(0)

    def isFinished(self) -> bool:
        return self.shoober.getPivotAngle() < 2

