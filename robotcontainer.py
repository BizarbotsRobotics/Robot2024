#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum

import commands2
from Subsystems.Intake.Intake import Intake
from Subsystems.ShooterClimber.ShooterClimber import ShooterClimber
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
        self.shooterclimber  = ShooterClimber()
        # self.testMotor = MotorController(MotorControllerType.SPARK_MAX, MotorType.BRUSHLESS, 9)
        self.driverController = wpilib.XboxController(0)
        
        # An example selectcommand. Will select from the three commands based on the value returned
        # by the selector method at runtime. Note that selectcommand takes a generic type, so the
        # selector does not have to be an enum; it could be any desired type (string, integer,
        # boolean, double...)
        # self.example_select_command = commands2.SelectCommand(
        #     # Maps selector values to commands
        #     {
        #         self.CommandSelector.ONE: commands2.PrintCommand(
        #             "Command one was selected!"
        #         ),
        #         self.CommandSelector.TWO: commands2.PrintCommand(
        #             "Command two was selected!"
        #         ),
        #         self.CommandSelector.THREE: commands2.PrintCommand(
        #             "Command three was selected!"
        #         ),
        #     },
        #     self.select,
        # )

        self.shooterclimber.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.RunCommand(
                lambda: self.shooterclimber.runClimber(
                    self.driverController.getLeftY()),
                self.shooterclimber
            )
        )

        # Configure the button bindings
        self.configureButtonBindings()

        



    def configureButtonBindings(self) -> None:
    
        #while(driverController.getAButtonPressed()):
        #self.testMotor.set(0.3)

        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kA).onTrue(
            commands2.InstantCommand(
                (lambda: self.shooterclimber.runShooter(-1)), self.shooterclimber
            )
        ).onFalse(
            commands2.InstantCommand(
                (lambda: self.shooterclimber.runShooter(0)), self.shooterclimber
                
            )
        )

        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kB).onTrue(
            commands2.InstantCommand(
                (lambda: self.intake.runIntake(.5)), self.intake
            )
        ).onFalse(
            commands2.InstantCommand(
                (lambda: self.intake.runIntake(0)), self.intake
                
            )
        )

        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kX).onTrue(
            commands2.InstantCommand(
                (lambda: self.conveyor.runConveyor(.5)), self.conveyor
            )
        ).onFalse(
            commands2.InstantCommand(
                (lambda: self.conveyor.runConveyor(0)), self.conveyor
                
            )
        )

        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kY).onTrue(
            commands2.InstantCommand(
                (lambda: self.shooterclimber.runIndexer()), self.shooterclimber
            )
        ).onFalse(
            commands2.InstantCommand(
                (lambda: self.shooterclimber.stopIndexer()), self.shooterclimber
                
            )
        )
        
        # commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kB).onTrue(
        #     commands2.InstantCommand(
        #         (lambda: self.testMotor.setPower(.5))
        #     )
        # ).onFalse(
        #     commands2.InstantCommand(
        #         (lambda: self.testMotor.setPower(0))
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