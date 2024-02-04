import math
from commands2 import Subsystem
import rev
import ntcore
from wpilib import AnalogEncoder, DriverStation
from wpimath import kinematics, geometry
import wpilib
from constants import ShooterConstants
from Util.MotorController import MotorController, MotorControllerType, MotorType
from enum import Enum

class ShooterClimber(Subsystem):

    def __init__(self):

        #Initalizes shooter & climber motors
        self.shooter1 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_BOTTOM)
        self.shooter2 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS,ShooterConstants.SHOOTER_TOP)

        self.shooter2.follow(self.shooter1)

        self.shooter1.setPIDValues(ShooterConstants.SHOOTER_FF, 
                                          ShooterConstants.SHOOTER_P, 
                                          ShooterConstants.SHOOTER_I, 
                                          ShooterConstants.SHOOTER_D, 
                                          ShooterConstants.SHOOTER_MAX_OUTPUT, 
                                          ShooterConstants.SHOOTER_MIN_OUTPUT)
       

        self.indexerMotorOne = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS,ShooterConstants.INDEXER_1)
        self.indexerMotorTwo = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS,ShooterConstants.INDEXER_2)

        self.indexerMotorOne.setPIDValues(ShooterConstants.INDEXER_FF, 
                                          ShooterConstants.INDEXER_P, 
                                          ShooterConstants.INDEXER_I, 
                                          ShooterConstants.INDEXER_D, 
                                          ShooterConstants.INDEXER_MAX_OUTPUT, 
                                          ShooterConstants.INDEXER_MIN_OUTPUT)

        

        

        self.indexerMotorOne.setEncoderPositionConversion(.33)
        #self.indexerMotorTwo.setEncoderPositionConversion(.2)
        
        self.indexerMotorTwo.follow(self.indexerMotorOne, False)
        self.shooterPivotMotor1 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_PIVOT_1)
        self.shooterPivotMotor2 = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ShooterConstants.SHOOTER_PIVOT_2)
        

        self.shooterPivotMotor2.follow(self.shooterPivotMotor1, True)


        self.shooterPivotMotor1.setPIDValues(ShooterConstants.SHOOTERPIVOT_FF, 
                                          ShooterConstants.SHOOTERPIVOT_P, 
                                          ShooterConstants.SHOOTERPIVOT_I, 
                                          ShooterConstants.SHOOTERPIVOT_D_D, 
                                          ShooterConstants.SHOOTERPIVOT_MAX_OUTPUT, 
                                          ShooterConstants.SHOOTERPIVOT_MIN_OUTPUT)
        
       
       # self.climberMotor = (MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, ClimberConstants.CLIMBER)

    # Function to set power on shooter motor
    def setShooterMotor(self, power):
        self.shooterMotor.setPower(power)

    def setShooterMotor_pid(self,speed):
        self.shooterMotor.setPower(speed)

    # Methods to set PID positions for the shooter
    def setShooterZero(self):
        self.shooterMotor.setShooterMotor_pid(ShooterConstants.SHOOTER_SPEEDS[0])

    def setShooterAmp(self):
        self.shooterMotor.setShooterMotor_pid(ShooterConstants.SHOOTER_SPEEDS[1])

    def setShooterLastResort(self):
        self.shooterMotor.setShooterMotor_pid(ShooterConstants.SHOOTER_SPEEDS[2])

    def setShooterHalfCourt(self):
        self.shooterMotor.setShooterMotor_pid(ShooterConstants.SHOOTER_SPEEDS[3])

    def setShooterFullSpeed(self):
        self.shooterMotor.setShooterMotor_pid(ShooterConstants.SHOOTER_SPEEDS[4])


    #Methods for the PID position of the pivot
    def setPivotZero(self):
        self.shooterPivotMotor1.setPivotPosition(ShooterConstants.SHOOTER_PIVOT_POSITIONS[0])

    def setPivotAmp(self):
        self.shooterPivotMotor1.setPivotPosition(ShooterConstants.SHOOTER_PIVOT_POSITIONS[1])

    def setPivotLastResort(self):
        self.shooterPivotMotor1.setPivotPosition(ShooterConstants.SHOOTER_PIVOT_POSITIONS[2])

    def setPivotHalfCourt(self):
        self.shooterPivotMotor1.setPivotPosition(ShooterConstants.SHOOTER_PIVOT_POSITIONS[3])
    


    

    def setPivotPosition(self, position):
        self.shooterPivotMotor1.setPosition(position)

    def runClimber(self, power):
        self.shooterPivotMotor1.setPower(power)

    def reverseClimber(self, power):
        pass

    def runIndexer(self):
        self.indexerMotorOne.setPower(-1)
        self.indexerMotorTwo.setPower(-1)

    def stopIndexer(self):
        self.indexerMotorOne.setPower(0)
        self.indexerMotorTwo.setPower(0)

    def runShooter(self, power):
        self.shooter1.setPower(power)

    def reverseShooter(self, power):
        pass
    
    def setIndexerPosition(self, position):
        self.indexerMotorOne.setPosition(position)



    





