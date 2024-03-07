import wpilib
import commands2
import commands2.cmd
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Intake.Intake import Intake


class ManualIntakeOutCmd(commands2.Command):
    """A command that will intake note to the conveyor, and hold it."""

    def __init__(self, conveyor:Conveyor, intake:Intake) -> None:
        self.conveyor = conveyor 
        self.intake = intake
        super().__init__()
        self.addRequirements(self.conveyor, self.intake)

    def initialize(self):
        self.conveyor.setConveyorPower(-.5)
        self.intake.setIntakePower(-1) 

    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self.conveyor.setConveyorPower(0)
        self.intake.setIntakePower(0) 

    def execute(self):
        self.conveyor.setConveyorPower(-.5)
        self.intake.setIntakePower(-1) 