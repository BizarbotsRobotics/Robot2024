import math
from commands2 import Subsystem
import rev
import ntcore
from wpilib import AnalogEncoder, DriverStation, PneumaticsModuleType
from wpimath import kinematics, geometry
import wpilib
from constants import ShooterConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType
from enum import Enum

class Shoober(Subsystem):

    def __init__(self):
        # Start smart dashboard
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startServer()
        self.sd = self.inst.getTable("SmartDashboard")

        self.distance = wpilib.DigitalInput(9)

        # self.pistonLock = wpilib.DoubleSolenoid(19, PneumaticsModuleType.REVPH, 0, 1)

        #Initalizes shooter & climber motors
        self.shooterMotorBottom = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_BOTTOM_ID)
        self.shooterMotorTop = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS,ShooterConstants.SHOOTER_TOP_ID)

        self.indexerMotorOne = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS,ShooterConstants.INDEXER_1_ID)
        self.indexerMotorTwo = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS,ShooterConstants.INDEXER_2_ID)

        self.shooterPivotMotorOne = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_PIVOT_1_ID)
        self.shooterPivotMotorTwo = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_PIVOT_2_ID)

        self.shooterPivotMotorOne.initAbsoluteEncoder()
        self.shooterPivotMotorOne.useAbsoluteEncoder()  
        self.shooterPivotMotorOne.getAbsoluteEncoder().setPositionConversionFactor(360)

        self.shooterMotorBottom.setPIDValues(ShooterConstants.SHOOTER_FF, 
                                          ShooterConstants.SHOOTER_P, 
                                          ShooterConstants.SHOOTER_I, 
                                          ShooterConstants.SHOOTER_D, 
                                          ShooterConstants.SHOOTER_MAX_OUTPUT, 
                                          ShooterConstants.SHOOTER_MIN_OUTPUT)

        self.indexerMotorOne.setPIDValues(ShooterConstants.INDEXER_FF, 
                                          ShooterConstants.INDEXER_P, 
                                          ShooterConstants.INDEXER_I, 
                                          ShooterConstants.INDEXER_D, 
                                          ShooterConstants.INDEXER_MAX_OUTPUT, 
                                          ShooterConstants.INDEXER_MIN_OUTPUT)
        
        self.indexerMotorTwo.setPIDValues(ShooterConstants.INDEXER_FF, 
                                          ShooterConstants.INDEXER_P, 
                                          ShooterConstants.INDEXER_I, 
                                          ShooterConstants.INDEXER_D, 
                                          ShooterConstants.INDEXER_MAX_OUTPUT, 
                                          ShooterConstants.INDEXER_MIN_OUTPUT)

        self.shooterPivotMotorOne.setPIDValues(ShooterConstants.SHOOTERPIVOT_FF, 
                                          ShooterConstants.SHOOTERPIVOT_P, 
                                          ShooterConstants.SHOOTERPIVOT_I, 
                                          ShooterConstants.SHOOTERPIVOT_D, 
                                          ShooterConstants.SHOOTERPIVOT_MAX_OUTPUT, 
                                          ShooterConstants.SHOOTERPIVOT_MIN_OUTPUT)

        self.shooterMotorTop.follow(self.shooterMotorBottom)
        self.shooterPivotMotorTwo.follow(self.shooterPivotMotorOne, True)

        self.indexerMotorOne.setEncoderPositionConversion(( .33 * 2.0 * 3.14 ))
        self.indexerMotorTwo.setEncoderPositionConversion(( .33 * 2.0 * 3.14 ))
        self.shooterPivotMotorOne.setEncoderPositionConversion(( 1.0 / 125.0) * 360)     

        self.shooterPivotMotorOne.setMotorBrake(True)
        self.shooterPivotMotorTwo.setMotorBrake(True)

        self.indexerMotorOne.setMotorBrake(True)
        self.indexerMotorTwo.setMotorBrake(True)
        
        self.shooterPivotMotorOne.setRampRate(20)
        self.shooterPivotMotorOne.setRampRate(20)

        self.indexerMotorOne.setMotorBrake(True)
        self.indexerMotorTwo.setMotorBrake(True)
        self.shooterMotorTop.save()
        self.shooterMotorBottom.save()
        self.indexerMotorOne.save()
        self.indexerMotorTwo.save()
        self.shooterPivotMotorOne.save()
        self.shooterPivotMotorTwo.save()
        self.noteHeld = False

    def periodic(self):
        self.telemetry()
        self.noteHeld = self.getNoteStoredUpdate()
        pass

    def telemetry(self):
        """
        Sends subsystem info to console or smart dashboard
        """
        self.sd.putNumber("Shoober RPM", self.getShooterRPM())
        self.sd.putNumber("Shoober Angle", self.shooterPivotMotorOne.getAbsoluteEncoderPosition() - 17.88)
        self.sd.putBoolean("Shooter Note Stored", self.getNoteStored())

        self.sd.putNumber("robot distance", self.distance.get())
        self.sd.putNumber("indexer position", self.getIndexerPosition())

        self.sd.putBoolean("AdjustNote", self.getIndexerPosition() < -4)

        pass

    # Function to set power on shooter motor
    def setShooterMotorPower(self, power):
        self.shooterMotorBottom.setPower(power)

    def setShooterMotorRPM(self,speed):
        self.shooterMotorBottom.setVelocity(speed)
  
    def getShooterRPM(self):
        return self.shooterMotorBottom.getBuiltInEncoderVelocity()
    
    def getPivotAngle(self):
        return self.shooterPivotMotorOne.getAbsoluteEncoderPosition() - 17.88
    
    def setPivotPower(self, power):
        if power > .05 or power < -.05:
            self.shooterPivotMotorOne.setPower(power)

    def setPivotPosition(self, position):
        self.shooterPivotMotorOne.setPosition(position + 17.88)

    def setDualIndexerPower(self, power):
        self.indexerMotorOne.setPower(power)
        self.indexerMotorTwo.setPower(power)

    def setTopIndexerPower(self, power):
        self.indexerMotorOne.setPower(power)
        
    def setBottomIndexerPower(self, power):
        self.indexerMotorTwo.setPower(power)
    
    def setDualIndexerPosition(self, position):
        self.indexerMotorOne.setPosition(position)
        self.indexerMotorTwo.setPosition(position)

    def setTopIndexerPosition(self, position):
        self.indexerMotorOne.setPosition(position)

    def setBottomIndexerPosition(self, position):
        self.indexerMotorTwo.setPosition(position)

    def getNoteStoredUpdate(self):
        if self.inst.getEntry("/proximity1").getDouble(0) > 150:
            return True
        return False
    
    def getNoteStored(self):
        return self.noteHeld
    
    def engageLock(self):
        self.pistonLock.set(wpilib.DoubleSolenoid.Value.kForward)

    def disengageLock(self):
        self.pistonLock.set(wpilib.DoubleSolenoid.Value.kReverse)

    def resetPosition(self):
        self.indexerMotorOne.resetPosition()
        self.indexerMotorTwo.resetPosition()

    def getIndexerPosition(self):
        return self.indexerMotorOne.getBuiltInEncoderPosition()

    

    





