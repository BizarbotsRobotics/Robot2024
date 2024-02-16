import math
import rev
import ntcore
from wpilib import AnalogEncoder, AnalogInput, DriverStation, RobotController
from wpimath import kinematics, geometry
import wpilib
from constants import SwerveConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType
from enum import Enum

class SwerveModule:
    """
    Swerve Module Class that represents a physical SDS swerve module. Use in conjuction with SwerveDrive.
    """
    def __init__(self, driveMotorId,swerveMotorId, absoluteEncoderPort, encoderOffset, motorControllerType, isInverted):
        # Start smart dashboard
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        self.sd = inst.getTable("SmartDashboard")

        self.swerveMotorId = swerveMotorId
        
        self.encoderOffset = encoderOffset
        self.maxSpeed = SwerveConstants.MAX_SPEED

        # Define motors for swerve drive
        self.swerveMotor = MotorController(motorControllerType, MotorType.BRUSHLESS, swerveMotorId)
        self.driveMotor =  MotorController(motorControllerType, MotorType.BRUSHLESS, driveMotorId)

        self.swerveMotor.setCurrentLimit(50)
        self.driveMotor.setCurrentLimit(50)

        #self.swerveMotor.setRampRate(10)

        # Get Absolute encoder
        self.swerveAbsoluteEncoder = AnalogEncoder(absoluteEncoderPort)

        self.swerveMotor.setInverted(isInverted)

        #self.swerveMotor.setPIDEncoder(self.swerveAbsoluteEncoder)
        
        
        

        # Set Encoder Conversion factors
        self.swerveMotor.setEncoderPositionConversion(SwerveConstants.SWERVE_ENCODER_CONVERSION_FACTOR)  
        #self.driveMotor.setEncoderPositionConversion(SwerveConstants.DRIVE_ENCODER_CONVERSION_FACTOR)
        # self.swerveMotor.setEncoderVelocityConversion(( SwerveConstants.SWERVE_ENCODER_CONVERSION_FACTOR * 360 ) / 60)  
        # self.driveMotor.setEncoderVelocityConversion(SwerveConstants.DRIVE_ENCODER_CONVERSION_FACTOR / 60)

       # self.swerveMotor.setEncoderInverted()       
        # Enable Position Wrapping From -180 to 180
        self.swerveMotor.setPositionPIDWrapping(True, -180, 180)

        # Seed the Swerve Relative encoder with the absolute encoders value
        


        # Set PID Values
        self.swerveMotor.setPIDValues(kf=SwerveConstants.SWERVE_FF, 
                                      kp=SwerveConstants.SWERVE_P, ki=SwerveConstants.SWERVE_I, 
                                      kd=SwerveConstants.SWERVE_D, kMinOut=SwerveConstants.SWERVE_MIN_OUTPUT, 
                                      kMaxOut=SwerveConstants.SWERVE_MAX_OUTPUT)

        # Set PID Values
        # self.driveMotor.setPIDValues(kf=SwerveConstants.DRIVE_FF, 
        #                               kp=SwerveConstants.DRIVE_P, ki=SwerveConstants.DRIVE_I, 
        #                               kd=SwerveConstants.DRIVE_D, kMinOut=SwerveConstants.DRIVE_MIN_OUTPUT, 
        #                               kMaxOut=SwerveConstants.DRIVE_MAX_OUTPUT)

        # Saves settings to motor controllers
        self.driveMotor.save()
        self.swerveMotor.save()

        reference = (self.getAbsolutePosition() - self.encoderOffset) * 360
        print(self.swerveMotor.seedBuiltInEncoder(self.getSwerveAbsolutePosition()))

        
        self.synchronizeEncoderQueued = True
        # Odometry Things
        self.lastState = self.getState()

    def setSwervePositionDegrees(self, degrees):
        """
        Sets the swerve motors position in degrees.

        Args:
            degrees (float): Position in degrees.
        """
        self.swerveMotor.setPosition(degrees)

    def setSwervePositionRadians(self, radians):
        """
        Sets the swerve motors position in radians.

        Args:
            radians (float): Position in radians.
        """
        degrees = radians/2*math.pi * 360
        self.setPositionDegrees(degrees)

    def getState(self):
        """
        Returns the current state of the swerve module.

        Returns:
            SwerveModuleState: The state of the module.
        """
        velocity = self.driveMotor.getBuiltInEncoderVelocity()
        azimuth = geometry.Rotation2d.fromDegrees(self.swerveMotor.getBuiltInEncoderPosition())
        return kinematics.SwerveModuleState(velocity, azimuth)

    def setDesiredState(self, desiredState, isOpenLoop=True):
        """
        Sets the state of the swerve module

        Args:
            desiredState (SwerveModuleState): Desired state of the Swerve Module.
            isOpenLoop (bool, optional): True is not PID controlled. Defaults to True.
        """
        #self.swerveMotor.seedBuiltInEncoder(self.getSwerveAbsolutePosition())
        desiredState = kinematics.SwerveModuleState.optimize(desiredState,
                                                geometry.Rotation2d.fromDegrees(self.getSwerveRelativePosition()))
        
        
        desiredState = self.antiJitter(desiredState, self.lastState, 10)

        # Azimuth Motor Set angle
        
        if (desiredState.angle is not self.lastState.angle) or self.synchronizeEncoderQueued:
            if self.swerveAbsoluteEncoder is not None and self.synchronizeEncoderQueued:
                absoluteEncoderPosition = (self.getAbsolutePosition()) * 360
                self.swerveMotor.seedBuiltInEncoder(self.getSwerveAbsolutePosition())
                self.setSwervePositionDegrees(desiredState.angle.degrees())
                self.synchronizeEncoderQueued = False   
                print("cheese")
            else:
                self.setSwervePositionDegrees(desiredState.angle.degrees())

        #Drive Motor
        if isOpenLoop:
            percentOutput = desiredState.speed / self.maxSpeed
            self.driveMotor.setPower(percentOutput)
        else:
            if desiredState.speed != self.lastState.speed:
                velocity = desiredState.speed
                self.driveMotor.setVelocity(velocity)

        #Odometry 
        self.lastState = desiredState

    def antiJitter(self,moduleState, lastModuleState, maxSpeed):
        """
        Perform anti-jitter within modules if the speed requested is too low.

        Args:
            moduleState (_type_): Desired Module State.
            lastModuleState (_type_): Previous Module State.
            maxSpeed (_type_): Max speed of the module.

        Returns:
            SwerveModuleState: New module State.
        """
        if abs(moduleState.speed) <= (maxSpeed * 0.01):
            moduleState.angle = lastModuleState.angle
        return moduleState

    def debug(self):
        """
        Sends debug info to condole or smart dashboard
        """
        self.sd.putNumber("Encoder "+ str(self.swerveMotorId), self.swerveMotor.getBuiltInEncoderPosition())
        self.sd.putNumber("Absolute Encoder "+ str(self.swerveMotorId), self.getSwerveAbsolutePositionReal())
        pass

    def getSwerveRelativePosition(self):
        """
        Returns the relative encoders position.

        Returns:
            float: Position in degrees of the built in encoder.
        """
        return self.swerveMotor.getBuiltInEncoderPosition()
    
    def getSwerveAbsolutePositionReal(self):
        return self.swerveAbsoluteEncoder.getAbsolutePosition()

    def getSwerveAbsolutePosition(self):
        """
        Returns the absolute encoders position.

        Returns:
            float: Position in degrees of the absolute encoder.
        """
        if self.swerveAbsoluteEncoder is not None:
            angle = (self.swerveAbsoluteEncoder.getAbsolutePosition())
            angle = angle - (self.encoderOffset)
            angle = angle *360
        angle %= 360
        if angle < 0.0:
            angle += 360
        return angle
    
    def getAbsolutePosition(self):
       return (self.swerveAbsoluteEncoder.getAbsolutePosition())

    def setMotorBrake(self, brake):
        """
        Sets the idle mode of the motor.

        Args:
            brake (Boolean): True for brake, False for coast.
        """
        self.driveMotor.setMotorBrake(brake)

    def queueSynchronizeEncoders(self):
        """
        Queue the encoders to be synced.
        """
        if self.swerveAbsoluteEncoder is not None:
            self.synchronizeEncoderQueued = True

    def setSwerveMotorVoltageCompenstion(self, voltage):
        """
        Set the voltage compensation for the swerve module motor.

        Args:
            voltage (float): Nominal voltage for operation to output to.
        """
        self.swerveMotor.setVoltageCompensation(voltage)


    def setDriveMotorVoltageCompenstion(self, voltage):
        """
        Set the voltage compensation for the swerve module motor.

        Args:
            voltage (float): Nominal voltage for operation to output to.
        """
        self.driveMotor.setVoltageCompensation(voltage)
    
    def getSwerveModulePosition(self):
        position = self.driveMotor.getBuiltInEncoderPosition()
        azimuth = geometry.Rotation2d.fromDegrees(self.getSwerveAbsolutePosition())
        return kinematics.SwerveModulePosition(position, azimuth)

    def pushOffsetsToControllers(self):
        print("rofl")
        if self.swerveAbsoluteEncoder is not None:
            print("kevin is dumb")
           # self.swerveAbsoluteEncoder.setPositionOffset(self.encoderOffset)
            print("lol")
            #self.encoderOffset = 0
            # else:
            #     print("Pushing the Absolute Encoder offset to the encoder failed on module #" + str(0), False)
        else:
            print("There is no Absolute Encoder on module #" + str(0), False)

    def setDriveInverted(self):
        self.driveMotor.setInverted(True)

        


