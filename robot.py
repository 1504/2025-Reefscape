import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import navx
import time
import drivesubsystem
import commands2
from commands2 import Command
import elevator
import constants
import intake
import commands2
from wpilib import SmartDashboard
from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.commands import FollowPathCommand

# To see messages from networktables, you must setup logging
import logging

import typing

logging.basicConfig(level=logging.DEBUG)

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.autonomousCommand: typing.Optional[commands2.Command] = None

        self.start_time = time.time()

        self.driver_controller = commands2.button.CommandXboxController(0)
        self.gadget_controller = commands2.button.CommandXboxController(1)
        self.swerve = drivesubsystem.DriveSubsystem()
        self.elevator_subsystem = elevator.ElevatorSubsystem()
        self.intake_subsystem = intake.IntakeSubsystem()

        #CameraServer.startAutomaticCapture("frontcam",0)
        

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)
        
        self.gadget_controller.a().whileTrue(elevator.ElevatorDownCommand(self.elevator_subsystem))
        self.gadget_controller.y().whileTrue(elevator.ElevatorL4Command(self.elevator_subsystem))
        self.gadget_controller.x().whileTrue(elevator.ElevatorL3Command(self.elevator_subsystem))
        self.gadget_controller.b().whileTrue(elevator.ElevatorL2Command(self.elevator_subsystem))
        self.gadget_controller.leftBumper().whileTrue(intake.PrimeCoralCommand(self.intake_subsystem))
        self.gadget_controller.leftTrigger().whileTrue(intake.BackCoralCommand(self.intake_subsystem))
        self.gadget_controller.rightBumper().whileTrue(intake.slowForwardCoralCommand(self.intake_subsystem))#slow corla
        self.gadget_controller.rightTrigger().whileTrue(intake.fastForwardCoralCommand(self.intake_subsystem))#fast coral

        # #self.gadget_controller.rightTri
        # gger().whileTrue(elevator.printHeightCommand(self.elevator_subsystem))

        self.gadget_controller.povUp().whileTrue(elevator.UpCommand(self.elevator_subsystem))
        self.gadget_controller.povDown().whileTrue(elevator.ElevatorDownManualCommand(self.elevator_subsystem))

        #Register Named Commands################################
        NamedCommands.registerCommand('ElevatorL2Command', elevator.ElevatorL2Command(self.elevator_subsystem))
        ##########################################################
        
        #Choosing an Autonomous Program########################
        self.defaultAuto = "Default"
        self.customAuto = "My Auto"
        self.chooser = AutoBuilder.buildAutoChooser()

        self.chooser.setDefaultOption("Default Auto", self.defaultAuto)
        self.chooser.addOption("My Auto", self.customAuto)
        SmartDashboard.putData("Auto choices", self.chooser)
        #Now, in autonomousInit and autonomousPeriodic, you can use the m_autoSelected variable to read which option was chosen, and change what happens during the autonomous period.

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        commands2.CommandScheduler.registerSubsystem(self.elevator_subsystem)
        commands2.CommandScheduler.registerSubsystem(self.intake_subsystem)
    
    ##################################################AUTON
    def autonomousInit(self) -> None:
        #commands2.CommandScheduler.getInstance().schedule(commands2.InstantCommand(lambda: self.swerve.drive(-0.2, 0, 0, False, True)))
        #self.autoSelected = self.chooser.getSelected()
        #print("Auto selected: " + self.autoSelected)

        #original: self.container.blah

        self.set_start_time()  # putting this after the scheduler is bad

        self.autonomousCommand = self.getAutonomousCommand()

        if self.autonomousCommand:
            commands2.CommandScheduler.getInstance().schedule(self.autonomousCommand)

    def autonomousPeriodic(self) -> None: 
        pass
        # match self.autoSelected:
        #     case self.customAuto:
        #         # Put custom auto code here
        #         pass
        #     case _:
        #         # Put default auto code here
        #         pass
    ##################################################AUTON

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        #if self.autonomousCommand:
            #self.autonomousCommand.cancel()
        pass

    def teleopPeriodic(self) -> None:
        # Teleop periodic logic
        self.driveWithJoystick(True)
        
    
    def testPeriodic(self) -> None:
        pass

    def driveWithJoystick(self, field_relative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        x_speed = (
            -self.x_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftY(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        y_speed = (
            -self.y_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftX(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rot_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getRightX(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )


        self.swerve.drive(x_speed, y_speed, rot, field_relative, rate_limit=True)

    #err stuff needed for auton pathplanner
    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def getAutonomousCommand(self):
        return self.chooser.getSelected()
    
if __name__ == "__main__":
    wpilib.run(MyRobot)