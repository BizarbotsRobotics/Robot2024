import math
import rev
import ntcore
from wpilib import AnalogEncoder, DriverStation
from wpimath import kinematics, geometry
import wpilib
from constants import ShooterConstants, ClimberConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType
from enum import Enum

class ShooterClimber:

    def __init__(self):

        #Initalizes shooter & climber motors
        self.shooter1 = MotorController(MotorControllerType.CANSparkMax, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_1)
        self.shooter2 = MotorController(MotorControllerType.CANSparkMax, MotorType.BRUSHLESS,ShooterConstants.SHOOTER_2)

        self.indexerMotor = MotorController(MotorControllerType.CANSparkMax, MotorType.BRUSHLESS,ShooterConstants.INDEXER)

        self.shooterPivotMotor1 = MotorController(MotorControllerType.CANSparkMax, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_PIVOT_1)
        self.shooterPivotMotor2 = MotorController(MotorControllerType.CANSparkMax, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_PIVOT_2)
        
        self.climberMotor = (MotorControllerType.CANSparkMax, MotorType.BRUSHLESS, ClimberConstants.CLIMBER)

    # Function to set power on shooter motor
    def setShooterMotor(self, power):
        self.shooterMotor.setPower(power)

    def setPivotPosition(self, position):
        pass

    def runClimber(self, power):
        pass
    def reverseClimber(self, power):
        pass

    def runShooter(self, power):
        pass

    def reverseShooter(self, power):
        pass


    





