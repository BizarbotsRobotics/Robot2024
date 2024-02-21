#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum

import commands2
from Commands.AdjustNoteCmd import AdjustNoteCmd
from Commands.AmpScore import AmpScore
from Commands.HoldNote import HoldNote
from Commands.IntakeCmd import IntakeCmd
from Commands.IntakeToShooter import IntakeToShooter
from Commands.Shoot import Shoot
from Commands.SpeakerShoot import SpeakerShoot
from Commands.StartConfig import StartConfig
from Subsystems.Drive.SwerveDrive import SwerveDrive
from Subsystems.Shoober.Shoober import Shoober
from Subsystems.Intake.Intake import Intake
from Subsystems.Conveyor.Conveyor import Conveyor
import commands2
from Util.MotorController import MotorController, MotorControllerType, MotorType
import wpimath.controller
import wpilib



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
        self.drive = SwerveDrive()
        self.driverController = wpilib.XboxController(0)
        self.operatorController = wpilib.XboxController(1)

        self.compressor = wpilib.Compressor(19,wpilib.PneumaticsModuleType.REVPH)
        self.compressor.disable()
        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive.driveFR(self.driverController.getLeftY(), self.driverController.getLeftX(), self.driverController.getRightX(), False, True), self.drive
            )
        )
        self.shoober.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.shoober.setPivotPower(
                    self.operatorController.getLeftY()),
                self.shoober
            )
        )

        # self.intake.setDefaultCommand(
        #     commands2.RunCommand(
        #         lambda: self.intake.setPivotPower(
        #             self.operatorController.getRightY()),
        #         self.intake
        #     )
        # )


        # Configure the button bindings
        self.configureButtonBindings()

        



    def configureButtonBindings(self) -> None:
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).onTrue(
            SpeakerShoot(self.shoober) 
            #Shoot(self.shoober)
        )

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).onTrue(
            IntakeToShooter(self.intake, self.conveyor, self.shoober)
           # AdjustNoteCmd(self.shoober)
        )

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).onTrue(
            StartConfig(self.shoober, self.intake)
        )
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).onTrue(
            AmpScore(self.shoober)

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
        return None