from Subsystems import Subsystem
from Util.MotorController import MotorController, MotorControllerType, MotorType
from constants import IntakeConstants
class Intake(Subsystem):

    def __init__(self):
        super().__init__()

        self.pivotMotor1 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.PIVOT_MOTOR_1)
        self.pivotMotor2 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.PIVOT_MOTOR_2)
        self.intakeWheelMotor = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.INTAKE_MOTOR)

        

    def setIntakePower(self, power):
        self.intakeMotor.setPower(power)

    def setPivotPosition(self, positionNum):
        