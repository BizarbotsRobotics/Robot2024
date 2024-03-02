import commands2
from Commands.AmpScore.AmpScoreCmd import AmpScoreCmd
from Commands.AmpScore.TurnOffPivotCmd import TurnOffPivotCmd
from Commands.Drive.SwerveAlignLimelightCmd import SwerveAlignLimelightCmd
from Commands.Shoot.SetShooterPivotCmd import SetShooterPivotCmd
from Commands.Shoot.ShootCmd import ShootCmd
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Vision.Vision import Vision


class AutoAmpScoreCmd(commands2.SequentialCommandGroup):

    def __init__(self, shoober: Shoober):
        super().__init__(
        )
        self.addCommands(AmpScoreCmd(shoober), TurnOffPivotCmd(shoober))