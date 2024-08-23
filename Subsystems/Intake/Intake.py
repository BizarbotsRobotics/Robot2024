import commands2
import ntcore
import wpilib
from Util.MotorController import MotorController, MotorControllerType, MotorType
from constants import IntakeConstants
from phoenix6.hardware import TalonFX
import phoenix6
import rev

class Intake(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        self.intakeRunning = False
        self.sd = inst.getTable("SmartDashboard")

        self.pivotMotorOne = TalonFX(IntakeConstants.PIVOT_MOTOR_1)
        self.pivotMotorTwo = TalonFX(IntakeConstants.PIVOT_MOTOR_2)
        self.intakeDown = False
        self.intakeMotor = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, IntakeConstants.INTAKE_MOTOR)

        self.cfg = phoenix6.configs.TalonFXConfiguration()
        self.cfg2 = phoenix6.configs.TalonFXConfiguration()
        self.cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        self.cfg2.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        self.request = phoenix6.controls.PositionVoltage(0)
        

        self.cfg.slot0.k_p = IntakeConstants.INTAKEPIVOT_P
        self.cfg.slot0.k_i = IntakeConstants.INTAKEPIVOT_I
        self.cfg.slot0.k_d = IntakeConstants.INTAKEPIVOT_D
        self.cfg2.slot0.k_g = IntakeConstants.INTAKEPIVOT_G

        self.pivotMotorTwo.set_control(self.request)
        self.pivotMotorTwo.set_control(phoenix6.controls.Follower(IntakeConstants.PIVOT_MOTOR_1, True))
        self.intakeMotor.setCurrentLimit(30)
        self.intakeMotor.save()
        self.pivotMotorOne.configurator.apply(self.cfg)
        self.pivotMotorTwo.configurator.apply(self.cfg2)
        self.pivotAbsoluteEncoder = wpilib.DutyCycleEncoder(8)
        self.encoderCount = 0
        self.setPivotEncoderVal(((self.pivotAbsoluteEncoder.getAbsolutePosition()) * 60))
        #self.resetPivotEncoder()

    def periodic(self):
        if self.encoderCount < 10:
            self.setPivotEncoderVal(((self.pivotAbsoluteEncoder.getAbsolutePosition()) * 60))
            self.encoderCount += 1

        self.telemetry()

    def telemetry(self):
        """
        Sends subsystem info to console or smart dashboard
        """
        pass
        self.sd.putNumber("Intake Angle Absolute", self.pivotAbsoluteEncoder.getAbsolutePosition())
        self.sd.putNumber("Intake Angle", self.getIntakePivotAngle())
        self.sd.putBoolean("Intake Running", self.intakeRunning)

    def getIntakeRPM(self):
        return self.intakeMotor.getBuiltInEncoderVelocity()

    def getIntakePivotAngle(self):
        return self.pivotMotorOne.get_rotor_position().value_as_double
    
    def setIntakePower(self, power):
        if power == 0:
            self.intakeRunning = False
        else:
            self.intakeRunning = True
        self.intakeMotor.setPower(power)

    def setPivotPosition(self, degrees):
        self.pivotMotorOne.set_control(self.request.with_position(degrees))

    def setPivotPower(self, power):
        self.pivotMotorOne.set_control(phoenix6.controls.DutyCycleOut(power))

    def setPivotEncoderVal(self, value):
        print(self.pivotMotorOne.set_position(value))
        print(self.pivotMotorTwo.set_position(value))

    def resetPivotEncoder(self):
        pass
        #self.pivotMotorOne.set_position(0)
