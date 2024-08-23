import wpilib
import commands2
import wpimath.controller
from Subsystems.Conveyor.Conveyor import Conveyor
from Subsystems.Shoober.Shoober import Shoober


class HoldNoteCmd(commands2.Command):
    """Brings the note to the shooter and primes it."""

    def __init__(self, shoober:Shoober, conveyor: Conveyor) -> None:
        self.shoober = shoober
        self.conveyor = conveyor
        self.run = True
        super().__init__()
        self.addRequirements(self.shoober, self.conveyor)

    def initialize(self):
        self.shoober.setPivotPosition(9)
        self.shoober.setShooterMotorPower(.05)

    def execute(self):
        if self.shoober.getPivotAngle() < 12:
            self.shoober.setDualIndexerPower(-.7)
            self.conveyor.setConveyorPower(1)

    def end(self, interrupted: bool):
        self.shoober.setDualIndexerPower(0)
        self.conveyor.setConveyorPower(0)
        self.shoober.setPivotPosition(0)

    def isFinished(self) -> bool:
        return self.shoober.getNoteStored()