import commands2
from Util.MotorController import MotorController, MotorControllerType, MotorType
from constants import IntakeConstants
class Intake(commands2.Subsystem):

    def __init__(self):
        super().__init__()

        self.pivotMotor1 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.PIVOT_MOTOR_1)
        self.pivotMotor2 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.PIVOT_MOTOR_2)

        self.intakeMotor = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.INTAKE_MOTOR)

        self.pivotMotor2.follow(self.pivotMotor1, True)

        self.pivotMotor1.setPIDValues(IntakeConstants._FF, 
                                          IntakeConstants.INAKEPIVOT_P, 
                                          IntakeConstants.INTAKEPIVOT_I, 
                                          IntakeConstants.INTAKEPIVOT_D, 
                                          IntakeConstants.INTAKEPIVOT_MAX_OUTPUT, 
                                          IntakeConstants.INTAKEPIVOT_MIN_OUTPUT)
       


        

    def runIntake(self, power):
        self.intakeMotor.setPower(power)

    def setPivotPosition(self, positionNum):
        pass