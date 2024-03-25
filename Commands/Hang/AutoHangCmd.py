import commands2
import commands2.cmd
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

class AutoHangCmd(commands2.Command):
    """A command that will set the shoober to a optimal postion for hanging."""

    def __init__(self, shoober: Shoober, intake: Intake) -> None:
        self.shoober = shoober
        self.intake = intake
        super().__init__()
        self.addRequirements(self.shoober, self.intake)

    def initialize(self):
        self.shoober.setPivotPower(-1)

    def execute(self):
        self.shoober.setPivotPower(-1)
        
    def end(self, interrupted: bool):
       self.shoober.setPivotPower(0)
       self.intake.setPivotPosition(33.5)

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  self.shoober.getPivotAngle() < 5
    