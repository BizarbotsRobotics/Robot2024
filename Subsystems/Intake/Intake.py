import commands2
import ntcore
from Util.MotorController import MotorController, MotorControllerType, MotorType
from constants import IntakeConstants
class Intake(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        # Start smart dashboard
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        self.sd = inst.getTable("SmartDashboard")

        # self.pivotMotorOne = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.PIVOT_MOTOR_1)
        # self.pivotMotorTwo = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.PIVOT_MOTOR_2)

        self.intakeMotor = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.INTAKE_MOTOR)

        # self.pivotMotorTwo.follow(self.pivotMotorOne, False)

        # self.pivotMotorOne.setPIDValues(IntakeConstants. 
        #                                   IntakeConstants.INAKEPIVOT_P, 
        #                                   IntakeConstants.INTAKEPIVOT_I, 
        #                                   IntakeConstants.INTAKEPIVOT_D, 
        #                                   IntakeConstants.INTAKEPIVOT_MAX_OUTPUT, 
        #                                   IntakeConstants.INTAKEPIVOT_MIN_OUTPUT)
        
        # self.pivotMotorOne.setEncoderPositionConversion((1.0/60.0) * 360.0)
        # self.pivotMotorOne.setMotorBrake(True)
        # self.pivotMotorTwo.setMotorBrake(True)
        self.intakeMotor.setCurrentLimit(30)
        # self.pivotMotorOne.save()
        # self.pivotMotorTwo.save()
        # self.intakeMotor.save()

    def periodic(self):
        self.telemetry()

    def telemetry(self):
        """
        Sends subsystem info to console or smart dashboard
        """
        self.sd.putNumber("Intake Angle", self.getIntakeRPM())
        self.sd.putNumber("Intake Speed", self.getIntakePivotAngle())

    def getIntakeRPM(self):
        return self.intakeMotor.getBuiltInEncoderVelocity()

    def getIntakePivotAngle(self):
        return self.pivotMotorOne.getBuiltInEncoderPosition()
    
    def setIntakePower(self, power):
        self.intakeMotor.setPower(power)

    def setPivotPosition(self, degrees):
        self.pivotMotorOne.setPosition(degrees)

    def setPivotPower(self, power):
        self.pivotMotorOne.setPower(power)