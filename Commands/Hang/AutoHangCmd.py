import commands2
import commands2.cmd
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

class AutoHangCmd(commands2.Command):
    """A command that will set the shoober to a optimal postion for hanging."""

    def __init__(self, shoober: Shoober) -> None:
        self.shoober = shoober
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        self.shoober.setPivotPosition(0)

    def execute(self):
        self.shoober.setPivotPosition(0)
        
    def end(self, interrupted: bool):
       self.shoober.engageLock()
       self.shoober.setPivotPower(0)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  self.shoober.getPivotAngle() < 5
    