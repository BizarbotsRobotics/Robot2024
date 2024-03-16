import commands2
import commands2.cmd
from Subsystems.Intake.Intake import Intake
from Subsystems.Shoober.Shoober import Shoober

class ToggleIntakeCmd(commands2.Command):
    """A command that will toggle the intake."""

    def __init__(self, intake: Intake) -> None:
        self.intake = intake
        super().__init__()
        self.addRequirements(self.intake)

    def initialize(self):
        print(self.intake.intakeDown)
        if self.intake.intakeDown:
            self.intake.intakeDown = False
            self.intake.setPivotPosition(32)
        else:
            self.intake.intakeDown = True
            self.intake.setPivotPosition(40.5)

    def execute(self):
        pass
        
    def end(self, interrupted: bool):
       pass

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return  True