#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum

import commands2
from Commands.AmpScore.AmpScoreCmd import AmpScoreCmd
from Commands.AmpScore.AutoAmpScoreCmd import AutoAmpScoreCmd
from Commands.Hang.AutoHangCmd import AutoHangCmd
from Commands.Hang.PrepareHangCmd import PrepareHangCmd
from Commands.Intake.IntakeToShooterCmd import IntakeToShooterCmd
from Commands.Shoot.DistanceShootCmd import DistanceShootCmd
from Commands.Shoot.ShootCloseCmd import ShootCloseCmd
from Commands.StartConfig.StartConfigCmd import StartConfigCmd
from Commands.Test.shooterTestCmd import ShooterTestCmd
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Intake.Intake import Intake
from Subsystems.Conveyor.Conveyor import Conveyor
import commands2
from Util.MotorController import MotorController, MotorControllerType, MotorType
import wpimath.controller
import wpilib
from pathplannerlib import path, auto
from pathplannerlib import auto, config
from Subsystems.Vision.Vision import Vision
from constants import AutoConstants



class RobotContainer:
    """This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """
    
    # The enum used as keys for selecting the command to run.
    class CommandSelector(enum.Enum):
        ONE = enum.auto()
        TWO = enum.auto()
        THREE = enum.auto()

    # An example selector method for the selectcommand.
    def select(self) -> CommandSelector:
        """Returns the selector that will select which command to run.
        Can base this choice on logical conditions evaluated at runtime.
        """
        return self.CommandSelector

    def __init__(self) -> None:
        self.conveyor = Conveyor()
        self.intake = Intake()
        self.shoober  = Shoober()
        self.vision = Vision()
        self.drive = SwerveDrive(self.vision)
        
        self.driverController = wpilib.XboxController(0)
        self.operatorController = wpilib.XboxController(1)

        self.compressor = wpilib.Compressor(19,wpilib.PneumaticsModuleType.REVPH)
        self.compressor.enableAnalog(30,60)
        
        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive.driveFR(self.driverController.getLeftY(), self.driverController.getLeftX(), self.driverController.getRightX(), False, True), self.drive
            )
        )
        
        # self.shoober.setDefaultCommand(
        #     ShooterTestCmd(self.shoober)
        # )

        # self.shoober.setDefaultCommand(
        #     commands2.RunCommand(
        #         lambda: self.shoober.setPivotPower(
        #             self.operatorController.getLeftY()),
        #         self.shoober
        #     )
        # )

        self.exampleTrigger = commands2.button.Trigger(self.vision.noteDetected)

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).onTrue(
            ShootCloseCmd(self.shoober)
        )

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kStart).onTrue(
            commands2.RunCommand(
                lambda: self.shoober.disengageLock(),
                    self.shoober
            )
        ) 

        self.exampleTrigger.onTrue(
            IntakeToShooterCmd(self.intake, self.conveyor, self.shoober)
        )

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).toggleOnTrue(
            IntakeToShooterCmd(self.intake, self.conveyor, self.shoober)
           # AdjustNoteCmd(self.shoober)
        )

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).onTrue(
            StartConfigCmd(self.intake, self.shoober)
        )
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).onTrue(
            AutoAmpScoreCmd(self.shoober)

        )

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).onTrue(
            PrepareHangCmd(self.shoober, self.intake)

        )

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).onTrue(
            AutoHangCmd(self.shoober)

        )
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).onTrue(
            
        #     commands2.RunCommand(
        #         lambda: self.shoober.engageLock(),
        #         self.shoober
        #     )
        # )
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).onTrue(
        #     commands2.RunCommand(
        #         lambda: self.shoober.disengageLock(),
        #         self.shoober
        #     )
        # )

        


        
        """Use this method to define your button->command mappings. Buttons can be created by
        instantiating a {GenericHID} or one of its subclasses
        ({edu.wpi.first.wpilibj.Joystick} or {XboxController}), and then calling passing it to a
        {edu.wpi.first.wpilibj2.command.button.JoystickButton}.
        """

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {Robot} class.

        :returns: the command to run in autonomous
        """
        return auto.AutoBuilder.followPath(path.PathPlannerPath.fromPathFile("New Path"))