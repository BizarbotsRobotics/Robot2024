import math
import rev
import ntcore
from wpilib import AnalogEncoder
from wpimath import kinematics, geometry
import wpilib
from constants import SwerveConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType

class SwerveModule:
    def __init__(self, driveMotorId,swerveMotorId, absoluteEncoderPort, encoderOffset, motorControllerType):

        # Start smart dashboard
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        self.sd = inst.getTable("SmartDashboard")
        
        self.encoderOffset = encoderOffset
        self.maxSpeed = SwerveConstants.MAX_SPEED

        # TODO figure out how to deal with this
        self.isOpenLoop = True

        # Define motors for swerve drive
        self.swerveMotor = MotorController(motorControllerType, MotorType.BRUSHLESS, swerveMotorId)
        self.driveMotor =  MotorController(motorControllerType, MotorType.BRUSHLESS, driveMotorId)

        # Get Absolute encoder
        self.swerveAbsoluteEncoder = AnalogEncoder(absoluteEncoderPort)
        
        # Set Encoder Conversion factors
        conversion =  (1.0 / ( 150.0 / 7.0 )) * 360
        self.swerveMotor.setEncoderPositionConversion(conversion)  

        # Enable Position Wrapping From -180 to 180
        self.swerveMotor.setPositionPIDWrapping(True, -180, 180)

        # Seed the Swerve Relative encoder with the absolute encoders value
        reference = self.getSwerveAbsolutePosition()
        self.swerveMotor.seedBuiltInEncoder(reference)

        # Define PID Values
        self.swervekP = .01
        self.swervekI = 0
        self.swervekD = 0
        self.swervekIz = 0
        self.swervekFF = 0
        self.swervekMinOutput = -1
        self.swervekMaxOutput = 1

        self.drivekP = 10
        self.drivekI = 0
        self.drivekD = 0
        self.drivekIz = 0
        self.drivekFF = 0
        self.drivekMinOutput = -1
        self.drivekMaxOutput = 1

        # Set PID Values
        self.swerveMotor.setPIDValues(kf=self.swervekFF, 
                                      kp=self.swervekP, ki=self.swervekI, 
                                      kd=self.swervekD, kMinOut=self.swervekMinOutput, 
                                      kMaxOut=self.swervekMaxOutput)

        # Set PID Values
        self.driveMotor.setPIDValues(kf=self.drivekFF, 
                                      kp=self.drivekP, ki=self.drivekI, 
                                      kd=self.drivekD, kMinOut=self.drivekMinOutput, 
                                      kMaxOut=self.drivekMaxOutput)

        # Saves settings to motor controllers
        self.driveMotor.save()
        self.swerveMotor.save()

        # Odometry Things
        self.lastState = self.getState()

    def setDrivePower(self, power):
        self.driveMotor.setPower(power)

    def setSwervePositionDegrees(self, degree):
        self.setPosition(degree)

    def setSwervePositionRadians(self, radian):
        setPoint = radian/2*math.pi
        self.setPosition(setPoint)
       
    def setPosition(self,setPoint):
        self.swerveMotor.setPosition(setPoint)

    def getState(self):
        velocity = self.driveMotor.getBuiltInEncoderVelocity()
        azimuth = geometry.Rotation2d.fromDegrees(self.swerveMotor.getBuiltInEncoderPosition())
        return kinematics.SwerveModuleState(velocity, azimuth)

    def setDesiredState(self, desiredState):

        desiredState = kinematics.SwerveModuleState.optimize(desiredState,
                                                geometry.Rotation2d.fromDegrees(self.getSwerveRelativePosition()))
        
        
        #desiredState = self.antiJitter(desiredState, self.lastState, 10)

        # Azimuth Motor Set angle
        self.setSwervePositionDegrees(desiredState.angle.degrees())

        #Drive Motor
        if self.isOpenLoop:
            percentOutput = desiredState.speed / self.maxSpeed
            self.driveMotor.setPower(percentOutput)
        else:
            # if desiredState.speed != self.lastState.speed:
            #     velocity = desiredState.speed_fps
            #     self.driveMotor.setReference(velocity, feedforward.calculate(velocity));
            pass
        #Odometry 
        self.lastState = desiredState

    def antiJitter(self,moduleState, lastModuleState, maxSpeed):
        if abs(moduleState.speed) <= (maxSpeed * 0.01):
            moduleState.angle = lastModuleState.angle
        return moduleState

    def debug(self):
        self.sd.putNumber("Encoder", self.swerveMotor.getBuiltInEncoderPosition())
        self.sd.putNumber("Absolute Encoder", self.swerveAbsoluteEncoder.getAbsolutePosition())
        pass

    def getSwerveRelativePosition(self):
        return self.swerveMotor.getBuiltInEncoderPosition()
    
    def getSwerveAbsolutePosition(self):
        if self.swerveAbsoluteEncoder is not None:
            angle = self.swerveAbsoluteEncoder.getAbsolutePosition() - self.encoderOffset
            angle = angle *360
        angle %= 360
        if angle < 0.0:
            angle += 360
        return angle
    
    def setMotorBrake(self, brake):
        self.driveMotor.setMotorBrake(brake)

        


