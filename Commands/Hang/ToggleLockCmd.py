import commands2
import commands2.cmd
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

class ToggleLockCmd(commands2.Command):
    """A command that will toggle the piston lock."""

    def __init__(self, shoober: Shoober) -> None:
        self.shoober = shoober
        super().__init__()
        self.addRequirements(self.shoober)

    def initialize(self):
        if self.shoober.pistonEnganged:
            self.shoober.disengageLock()
        else:
            self.shoober.engageLock()

    def execute(self):
        pass
        
    def end(self, interrupted: bool):
       pass

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  True