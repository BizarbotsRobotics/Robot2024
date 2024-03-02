import wpilib
import commands2
import commands2.cmd
import wpimath.controller
from Subsystems.Shoober.Shoober import Shoober
import constants


class TurnOffPivotCmd(commands2.Command):
    """A command that will score a note into the Amp."""

    def __init__(self, shoober: Shoober) -> None:
        self.shoober = shoober
        super().__init__()
        self.addRequirements(self.shoober)
        

    def initialize(self):
        # self.shoober.setShooterMotorRPM(-3000)
        # self.shoober.setPivotPosition(120)
        self.shoober.setPivotPower(0)

    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        pass
        


    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return True