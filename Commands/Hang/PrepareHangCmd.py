import commands2
import commands2.cmd
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

class PrepareHangCmd(commands2.Command):
    """A command that will set the shoober to a optimal postion for hanging."""

    def __init__(self, shoober: Shoober, intake: Intake) -> None:
        self.shoober = shoober
        self.intake = intake
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        self.intake.setPivotPosition(30)
        self.shoober.disengageLock()
        self.shoober.setPivotPosition(100)

    def execute(self):
        self.intake.setPivotPosition(30)
        self.shoober.setPivotPosition(100)
        pass
        
    def end(self, interrupted: bool):
       pass


    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  self.shoober.getPivotAngle() > 97