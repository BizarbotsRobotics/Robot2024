import math
import rev
import ntcore
from wpilib import AnalogEncoder, DriverStation
from wpimath import kinematics, geometry
import wpilib
from constants import SwerveConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType
from enum import Enum

class Conveyor:

    def _init_(self,conveyorMotorId, absoluteEncoderPort, encoderOffset, motorControllerType):
        self.conveyorMotor = MotorController(motorControllerType, MotorType.BRUSHLESS, conveyorMotorId)
        
    # Sets the absolute encoder port
    INTAKE_ENCODER_PORT = 4



    def setConveyorPower(self, conveyorMotor, power):
        self.conveyorMotor.setPower(power)

    

