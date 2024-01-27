import math
from Util.MotorController import MotorController, MotorControllerType, MotorType
import wpilib
import ntcore
from wpimath import kinematics, geometry

#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import typing

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
#hello abby this is a test

# class Robot(wpilib.TimedRobot):
#     counter = 0

#     def robotInit(self):
#         #self.limelight = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
#         self.joy = wpilib.XboxController(0)
#         self.swerveDrive = SwerveDrive()
        

#     def robotPeriodic(self):
#         self.swerveDrive.enableTelemetry()
#         #print(self.limelight.getNumber("tx", 0.0))
#         pass

#     def autonomousInit(self):
#         pass

#     def autonomousPeriodic(self):
#         pass

#     def teleopInit(self):
#         pass


#     def teleopPeriodic(self):
#         y = self.joy.getLeftY()*15

#         velocity = kinematics.ChassisSpeeds(-self.joy.getLeftX()*15, y, self.joy.getRightX()*15)
#         self.swerveDrive.drive(velocity, True, geometry.Translation2d(0,0))




# if __name__ == "__main__":
#     wpilib.run(Robot)