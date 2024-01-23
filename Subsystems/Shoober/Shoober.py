import math
import rev
import ntcore
from wpilib import AnalogEncoder, DriverStation
from wpimath import kinematics, geometry
import wpilib
from constants import SwerveConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType
from enum import Enum

class Shoober:

    def _init_(self,shooberMotorId, absoluteEncoderPort, encoderOffset, motorControllerType):

        #Initalizes shooter & climber motors
        self.frontShooter = MotorController(motorControllerType, MotorType.BRUSHLESS, shooberMotorId)
        self.backShooter = MotorController(motorControllerType, MotorType.BRUSHLESS, shooberMotorId)
        self.indexerMotor = MotorController(motorControllerType, MotorType.BRUSHLESS, shooberMotorId)
        self.pivotMotor1 = MotorController(motorControllerType, MotorType.BRUSHLESS, shooberMotorId)
        self.pivotMotor2 = MotorController(motorControllerType, MotorType.BRUSHLESS, shooberMotorId)

        # Sets the shoober absolute encoder port
        SHOOBER_ENCODER_PORT = 5

        # Function to set shoober controller
        def setShooberMotor(self, shooberMotor, power):
            self.shooberMotor.setPower(power)


