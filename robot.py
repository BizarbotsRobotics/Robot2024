import math
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Drive.SwerveModule import SwerveModule
from Util.MotorController import MotorController, MotorControllerType, MotorType
import wpilib
import ntcore
from wpimath import kinematics, geometry

#hello abby this is a test

class Robot(wpilib.TimedRobot):
    counter = 0

    def robotInit(self):
        #self.limelight = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        self.joy = wpilib.XboxController(0)
        self.swerveDrive = SwerveDrive()
        self.counter = 0
        

    def robotPeriodic(self):
        self.swerveDrive.enableTelemetry()
        #print(self.limelight.getNumber("tx", 0.0))
        pass

    def autonomousInit(self):
        pass


    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass


    def teleopPeriodic(self):
        y = self.joy.getLeftY()*15

        velocity = kinematics.ChassisSpeeds(-self.joy.getLeftX()*15, y, self.joy.getRightX()*15)
        self.swerveDrive.drive(velocity, True, geometry.Translation2d(0,0))




if __name__ == "__main__":
    wpilib.run(Robot)