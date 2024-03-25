import commands2
import commands2.cmd
from Subsystems.Intake.Intake import Intake


class IntakeStartPositionCmd(commands2.Command):
    """A command that will take the robot out of start config."""

    def __init__(self, intake: Intake) -> None:
        self.intake = intake
        super().__init__()
        self.addRequirements(self.intake)

    def initialize(self):
        pass
            
    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return self.intake.getIntakePivotAngle() < 25

    def execute(self):
        self.intake.setPivotPosition(18.6)
        