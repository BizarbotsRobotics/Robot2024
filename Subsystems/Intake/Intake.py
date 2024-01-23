import math
import rev
import ntcore
from wpilib import AnalogEncoder, DriverStation
from wpimath import kinematics, geometry
import wpilib
from constants import SwerveConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType
from enum import Enum

class Intake:

    def _init_(self, intakeMotorId, absoluteEncoderPort, encoderOffset, motorControllerType):

        self.pivotMotor1 = MotorController(motorControllerType, MotorType.BRUSHLESS, intakeMotorId)
        self.pivotMotor2 = MotorController(motorControllerType, MotorType.BRUSHLESS, intakeMotorId)
        self.intakeWheelMotor = MotorController(motorControllerType, MotorType.BRUSHLESS, intakeMotorId)

        

        def setIntakePower(self, intakeMotor, power):
            self.intakeMotor.setPower(power)