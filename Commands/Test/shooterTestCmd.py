import wpilib
import commands2
import commands2.cmd
import wpimath.controller
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober
from wpilib.shuffleboard import Shuffleboard


class ShooterTestCmd(commands2.Command):
    """A command that will take the robot out of start config."""

    def __init__(self, shoober: Shoober) -> None:
        self.shoober = shoober
        tab = Shuffleboard.getTab("Shooter Testing")
        self.position = tab.add("Set Pivot Postion", 0).getEntry()
        self.rpm = tab.add("Set RPM", 0).getEntry()
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        pass

    def execute(self):
        self.shoober.setPivotPosition(self.position.getDouble(0.0))
        self.shoober.setShooterMotorRPM(self.rpm.getDouble(0.0))
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False

