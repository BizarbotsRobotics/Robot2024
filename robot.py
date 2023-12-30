import math
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Drive.SwerveModule import SwerveModule
from Util.MotorController import MotorController, MotorControllerType, MotorType
import wpilib
from wpimath import kinematics, geometry

#hello abby this is a test

class Robot(wpilib.TimedRobot):
    counter = 0

    def robotInit(self):
        

        self.joy = wpilib.XboxController(0)

        self.swerveDrive = SwerveDrive()
        self.swerveModuleFrontRight = SwerveModule(1,2,1)
        

    def robotPeriodic(self):
        self.swerveModuleFrontRight.debug()
        pass

    def autonomousInit(self):
        pass


    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass


    def teleopPeriodic(self):
        y = self.joy.getLeftY()*15

        velocity = kinematics.ChassisSpeeds(y, self.joy.getLeftX()*15, self.joy.getRightX()*15);
        swerveModuleStates = self.kinematics.toSwerveModuleStates(velocity, geometry.Translation2d());

        self.swerveModuleFrontRight.setDesiredState(swerveModuleStates[2]);
        #setRawModuleStates(swerveModuleStates, isOpenLoop);
        #self.swerveModuleFrontRight.setPosition(1.5)
        #self.swerveModuleFrontRight.printAbsEncoder()




if __name__ == "__main__":
    wpilib.run(Robot)