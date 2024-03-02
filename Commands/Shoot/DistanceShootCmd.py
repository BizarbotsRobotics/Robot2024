import commands2
from Commands.Drive.SwerveAlignLimelightCmd import SwerveAlignLimelightCmd
from Commands.Shoot.SetShooterPivotCmd import SetShooterPivotCmd
from Commands.Shoot.ShootCmd import ShootCmd
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Vision.Vision import Vision


class DistanceShootCmd(commands2.SequentialCommandGroup):

    def __init__(self, vision: Vision, drive: SwerveDrive, shoober: Shoober):
        super().__init__(
        )
        self.addCommands(SwerveAlignLimelightCmd(drive, vision), SetShooterPivotCmd(shoober, vision), ShootCmd(shoober, vision))