import rev
import ctre
from enum import Enum
 
class MotorControllerType(Enum):
    SPARK_MAX = 1
    FALCON = 2
    VICTOR_SPX = 3
    TALON_SRX = 4

class MotorType(Enum):
    BRUSHED = 1
    BRUSHLESS = 2
 
    

class MotorController:
    """
    Generic Motor Controller class that allows the same class to be used for a multitude of motor controllers.
    """
    def __init__(self, motorControllerType, motorType, motorID):
        """

        Args:
            motorControllerType (_type_): Brand/Device Name.
            motorType (_type_): Brushed or Brushless
            motorID (_type_): CAN ID of the motor

        Raises:
            ValueError: Incorrect motor controller type.
        """
        self.motorID = motorID
        self.motorType = motorType
        self.motorControllerType = motorControllerType


        if motorControllerType == MotorControllerType.SPARK_MAX:
            self.motor = rev.CANSparkMax(motorID,rev.CANSparkMax.MotorType.kBrushless)
            self.encoder = self.motor.getEncoder()
            self.pidController = self.motor.getPIDController()
        elif motorControllerType == MotorControllerType.FALCON:
            self.motor = ctre.TalonFX(motorID)
        elif motorControllerType == MotorControllerType.VICTOR_SPX:
            self.motor = ctre.VictorSPX(motorID)
        elif motorControllerType == MotorControllerType.TALON_SRX:
            self.motor = ctre.TalonSRX(motorID)
        else:
             raise ValueError("Pick correct motor loser dummy idiot! 😡")
        
        self.reset()
        self.setCurrentLimit()
        self.setPIDEncoder()

    def setPower(self, power):
        """
        Sets the power of the motor as a percentage between -1.0 and 1.0.

        Args:
            power (float): Power of motor to be set.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.motor.set(power)
        else:
            self.motor.set(ctre.ControlMode.PercentOutput,power)    

    def setPosition(self, position):
        """
        Sets the position of the motor using the integrated PID controller.

        Args:
            position (float): position to be set in rotations or in converted units.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.__getPIDController__().setReference(position, rev.CANSparkMax.ControlType.kPosition)
        else:
            self.motor.set(ctre.ControlMode.Position, position)

    def setVelocity(self, velocity):
        """
        Sets the velocity of the motor using the integrated PID controller.

        Args:
            velocity (_type_): volocity to be set in rpm or converted units.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.__getPIDController__().setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)
        else:
            self.motor.set(ctre.ControlMode.Velocity, rpm)

    def __getPIDController__(self):
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            return self.pidController
        else:
            return None
        
    def setPIDValues(self, kf=0.0, kp=0.0, ki=0.0, kd=0.0, kMaxOut=1.0, kMinOut=-1.0):
        """
        Sets the pid values using the integrated PID controller.

        Args:
            kf (float, optional): Feed Foward Value. Defaults to 0.0.
            kp (float, optional): Proportional gain value. Defaults to 0.0.
            ki (float, optional): Integral Value. Defaults to 0.0.
            kd (float, optional): Derivitive Value. Defaults to 0.0.
            kMaxOut (float, optional): Max Output. Defaults to 1.0.
            kMinOut (float, optional): Min Output. Defaults to -1.0.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.__getPIDController__().setFF(kf)
            self.__getPIDController__().setP(kp)
            self.__getPIDController__().setI(ki)
            self.__getPIDController__().setD(kd)
            self.__getPIDController__().setOutputRange(kMinOut, kMaxOut)
        else:
            self.motor.selectProfileSlot(0, 0)
            self.motor.config_kF(0, kf)
            self.motor.config_kP(0, kp)
            self.motor.config_kI(0, ki)
            self.motor.config_kD(0, kd)
            self.motor.configNominalOutputForward(0, self.kTimeoutMs)
            self.motor.configNominalOutputReverse(0, self.kTimeoutMs)
            self.motor.configPeakOutputForward(kMaxOut, self.kTimeoutMs)
            self.motor.configPeakOutputReverse(kMinOut, self.kTimeoutMs)
    
    def getBuiltInEncoderPosition(self):
        """
        Returns the built in encoders position.

        Returns:
            float: position in rotations or converted unit.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            return self.encoder.getPosition()
        else:
            return self.motor.getSelectedSensorPosition(0)
    
    def getBuiltInEncoderVelocity(self):
        """
        Returns the built in encoders velocity.

        Returns:
            float: position in rotations or converted unit.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            return self.encoder.getVelocity()
        else:
            return self.motor.getSelectedSensorVelocity(0)
        
    def setEncoderPositionConversion(self, conversion):
        """
        Sets the position conversion factor for the built-in encoder.

        Args:
            conversion (float): Conversion factor.
        """
        self.positionConversion = conversion
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.encoder.setPositionConversionFactor(conversion)

    def setEncoderVelocityConversion(self, conversion):
        """
        Sets the velocity conversion factor for the built-in encoder.

        Args:
            conversion (float): Conversion factor.
        """
        self.PositionConversion = conversion
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.swerveRelativeEncoder.setVelocityConversionFactor(conversion)

    def reset(self):
        """
        Resets the motor controller to factory settings. This should be called everytime you make a motor controller.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.motor.restoreFactoryDefaults()
        else:
            self.motor.configFactoryDefault()

    def save(self):
        """
        Saves any settings that were changed in the motor controller, should be called when settings have been changed.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.motor.burnFlash()

    def seedBuiltInEncoder(self, position):
        """
        Sets the built in encoders value to the given position value.

        Args:
            position (float): position to be set.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.encoder.setPosition(position)
        else:
            self.motor.setSelectedSensorPosition(position)

    def setCurrentLimit(self, limit=30):
        """
        Sets the current limit on the motor, make sure to set this to fix battery and motor burnout issues.

        Args:
            limit (int, optional): Current value to be set. Defaults to 40.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.motor.setSmartCurrentLimit(limit)
        else:
            self.motor.configPeakCurrentDuration(1.5)
            self.motor.configPeakCurrentLimit(limit)
            self.motor.configStatorCurrentLimit(limit)

    def setPositionPIDWrapping(self, enable=False, min=-180, max=180):
        """
        Enables position wrapping.

        Args:
            enable (bool, optional): _description_. Defaults to False.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.__getPIDController__().setPositionPIDWrappingEnabled(enable)
            self.__getPIDController__().setPositionPIDWrappingMinInput(min)
            self.__getPIDController__().setPositionPIDWrappingMaxInput(max)
        #TODO Implement with CTRE devices
        self.positionWrapping = True

    def setPIDEncoder(self, encoder=None):
        """
        Sets the encoder to be used in the integrated PID Controller

        Args:
            encoder (Encoder, optional): External encode to be used in the PID controller. Defaults to None.
        """
        if encoder is None:
            if self.motorControllerType is MotorControllerType.SPARK_MAX:
                self.__getPIDController__().setFeedbackDevice(self.encoder)
            else:
                self.motor.configSelectedFeedbackSensor(
                    ctre.FeedbackDevice.CTRE_MagEncoder_Relative,
                    self.kPIDLoopIdx,
                    self.kTimeoutMs,
                )
                self.motor.setSensorPhase(True)
                self.motor.setInverted(False)
        else:
            if self.motorControllerType is MotorControllerType.SPARK_MAX:
                self.__getPIDController__().setFeedbackDevice(encoder)
            else:
                self.motor.configSelectedFeedbackSensor(encoder, 0, 10)
                self.motor.setSensorPhase(True)
                self.motor.setInverted(False)
    
    def setVoltageCompensation(self, voltage):
        """
        Sets the voltage compensation on the motor. You should set this for more reliable outcomes.

        Args:
            voltage (float): Voltage compensation value.
        """
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            self.motor.enableVoltageCompensation(voltage)
        else:
            self.motor.configVoltageCompSaturation(voltage)
    

    def setMotorBrake(self, brake):
        """
        Sets the idle mode of the motor.

        Args:
            brake (Boolean): True sets the brake on, False sets the brake off.
        """
    
        if self.motorControllerType is MotorControllerType.SPARK_MAX:
            if brake:
                self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
            else: 
                self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        else:
            if brake:
                self.motor.setNeutralMode(ctre.NeutralMode.Brake)
            else: 
                self.motor.setNeutralMode(ctre.NeutralMode.Coast)


