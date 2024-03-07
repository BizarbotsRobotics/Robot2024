#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum
import math

import commands2
from Commands.AmpScore.AmpScoreCmd import AmpScoreCmd
from Commands.AmpScore.AutoAmpScoreCmd import AutoAmpScoreCmd
from Commands.Autos.TwoNote.TwoNoteAutoCmd import TwoNoteAutoCmd
from Commands.Drive.DriveAmpCmd import DriveAmpCmd
from Commands.Drive.DriveSpeakerCmd import DriveSpeakerCmd
from Commands.Drive.DriveCmd import DriveCmd
from Commands.Hang.AutoHangCmd import AutoHangCmd
from Commands.Hang.PrepareHangCmd import PrepareHangCmd
from Commands.Hang.ToggleLockCmd import ToggleLockCmd
from Commands.Intake.IntakeToShooterCmd import IntakeToShooterCmd
from Commands.Intake.ManualIntakeInCmd import ManualIntakeInCmd
from Commands.Intake.ManualIntakeOutCmd import ManualIntakeOutCmd
from Commands.Intake.ToggleIntakeCmd import ToggleIntakeCmd
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
import wpimath.geometry as geometry
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

    class DriveSelector(enum.Enum):
        DRIVE = enum.auto()
        DRIVESPEAKER = enum.auto()
        DRIVEAMP = enum.auto()

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

        # Drive Command = Drive Controller
        self.drive.setDefaultCommand(DriveCmd(self.drive, self.driverController.getLeftY, self.driverController.getLeftX, self.driverController.getRightX))
            # commands2.selectcommand.SelectCommand(
            #     {
            #        self.DriveSelector.DRIVE: DriveCmd(self.drive, self.driverController.getLeftY, self.driverController.getLeftX, self.driverController.getRightX),
            #        self.DriveSelector.DRIVESPEAKER: DriveSpeakerCmd(self.drive, self.driverController.getLeftY, self.driverController.getLeftX, self.driverController.getRightX),
            #        self.DriveSelector.DRIVEAMP: DriveAmpCmd(self.drive, self.driverController.getLeftY, self.driverController.getLeftX, self.driverController.getRightX)
            #     }, self.getButtonDrive
            # )
            # commands2.RunCommand(
            #     lambda: self.drive.driveFR(self.driverController.getLeftY(), self.driverController.getLeftX(), self.driverController.getRightX(), False, True), self.drive
            # )
        #)

        

        # self.shoober.setDefaultCommand(
        #     ShooterTestCmd(self.shoober)
        # )

        self.shoober.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.shoober.setPivotPower(
                    self.operatorController.getLeftY()),
                self.shoober
            )
        )

        self.noteTrigger = commands2.button.Trigger(self.vision.noteDetected)
        self.opLeftYTrigger = commands2.button.Trigger(lambda: not (self.operatorController.getLeftY() < .1 and self.operatorController.getLeftY() > -.1))
        self.opRightYTrigger = commands2.button.Trigger(lambda: not (self.operatorController.getRightY() < .1 and self.operatorController.getRightY() > -.1))
        self.opBackTrigger = commands2.button.Trigger(lambda: not ( self.operatorController.getRightTriggerAxis() > .1 and self.operatorController.getRightTriggerAxis() > .1))

        # Configure the button bindings
        self.configureButtonBindings()
        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption("One Note Drive Straight", None)
        self.chooser.addOption("Two Note", TwoNoteAutoCmd(self.drive, self.shoober, self.intake, self.conveyor))

        wpilib.SmartDashboard.putData("auto", self.chooser)

    def getButtonDrive(self):
        if self.driverController.getAButtonPressed():
            return self.DriveSelector.DRIVESPEAKER
        elif self.driverController.getBButton():
            return self.DriveSelector.DRIVEAMP
        else:
            return self.DriveSelector.DRIVE
        
    def configureButtonBindings(self) -> None:

        # Shoot Speaker Close - X Button Operator
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).onTrue(
            ShootCloseCmd(self.shoober)
        )

        # Intake Command - A Button Operator
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).toggleOnTrue(
            IntakeToShooterCmd(self.intake, self.conveyor, self.shoober)
        )

        # Amp Score Command - B Button Operator
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).onTrue(
            AutoAmpScoreCmd(self.shoober)
        )

        # Shoot Speaker Far - Y Button Operator
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).onTrue(
            DistanceShootCmd(self.vision, self.drive, self.shoober)
        )

        # Toggle Piston Lock - Back Button Operator
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kBack).onTrue(
            ToggleLockCmd(self.shoober)
        ) 

        # Toggle Intake Position - Start Button Operator
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kStart).toggleOnTrue(
            ToggleIntakeCmd(self.intake)
        ) 

        # Prepare Climb Command - POV Up Operator
        commands2.button.povbutton.POVButton(self.operatorController, 0).onTrue(
            PrepareHangCmd(self.shoober, self.intake)
        )

        # Climb Command - POV Down Operator
        commands2.button.povbutton.POVButton(self.operatorController, 180).onTrue(
            AutoHangCmd(self.shoober)
        )

        # Manual Pivot Command - Right Y axis
        self.opRightYTrigger.onTrue(
            commands2.RunCommand(
                lambda: self.shoober.setPivotPower(
                    self.operatorController.getRightY()),
                self.shoober
            )
        )

        # Manual Pivot Command - Left Y axis
        self.opLeftYTrigger.onTrue(
            commands2.RunCommand(
                lambda: self.shoober.setDualIndexerPower(
                    self.operatorController.getLeftY()),
                self.shoober
            )
        )

        # Manual Intake In - Right Back Bumper
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).whileTrue(
            ManualIntakeInCmd(self.conveyor, self.intake)
        ) 

        # Manual Intake Out - Left Back Bumper
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).whileTrue(
            ManualIntakeOutCmd(self.conveyor, self.intake)
        ) 

        # Manual Shooter Command - Back Trigger
        self.opBackTrigger.onTrue(
            commands2.RunCommand(
                lambda: self.shoober.setShooterMotorPower(
                    self.operatorController.getRightTriggerAxis() - self.operatorController.getLeftTriggerAxis() ),
                self.shoober
            )
        )

        # Auto Intake Command - Machine Learning
        self.noteTrigger.onTrue(
            IntakeToShooterCmd(self.intake, self.conveyor, self.shoober)
        )
        
        """Use this method to define your button->command mappings. Buttons can be created by
        instantiating a {GenericHID} or one of its subclasses
        ({edu.wpi.first.wpilibj.Joystick} or {XboxController}), and then calling passing it to a
        {edu.wpi.first.wpilibj2.command.button.JoystickButton}.
        """

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {Robot} class.

        :returns: the command to run in autonomous
        """
        return self.chooser.getSelected()
    