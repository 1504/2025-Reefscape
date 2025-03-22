import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import navx
import drivesubsystem
import commands2
import elevator
import constants
import intake
import algae
from wpilib import Timer

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        self.driver_controller = commands2.button.CommandXboxController(0)
        self.gadget_controller = commands2.button.CommandXboxController(1)
        self.swerve = drivesubsystem.DriveSubsystem()
        self.elevator_subsystem = elevator.ElevatorSubsystem()
        self.intake_subsystem = intake.IntakeSubsystem()
        self.algae_subsystem = algae.AlgaeSubsystem()
        #CameraServer.startAutomaticCapture("frontcam",0)
        

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)


        # Algae Bindings
        self.gadget_controller.povRight().whileTrue(algae.inwardClawCommand(self.algae_subsystem))
        self.gadget_controller.povUp().whileTrue(algae.holdAlgaeCommand(self.algae_subsystem))
        self.gadget_controller.povLeft().whileTrue(algae.outwardClawCommand(self.algae_subsystem))

        # elevator bindings
        self.gadget_controller.a().whileTrue(elevator.ElevatorDownCommand(self.elevator_subsystem))
        self.gadget_controller.y().whileTrue(elevator.ElevatorL4Command(self.elevator_subsystem))
        self.gadget_controller.x().whileTrue(elevator.ElevatorL3Command(self.elevator_subsystem))
        self.gadget_controller.b().whileTrue(elevator.ElevatorL2Command(self.elevator_subsystem))
        # self.gadget_controller.povUp().whileTrue(elevator.UpCommand(self.elevator_subsystem))
        # self.gadget_controller.povDown().whileTrue(elevator.ElevatorDownManualCommand(self.elevator_subsystem)) 
        commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() < -0.5).whileTrue(elevator.UpCommand(self.elevator_subsystem))
        commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() > 0.5).whileTrue(elevator.ElevatorDownManualCommand(self.elevator_subsystem))


        # coral Bindings
        self.gadget_controller.leftBumper().whileTrue(intake.PrimeCoralCommand(self.intake_subsystem))
        self.gadget_controller.leftTrigger().whileTrue(intake.BackCoralCommand(self.intake_subsystem))
        self.gadget_controller.rightBumper().whileTrue(intake.slowForwardCoralCommand(self.intake_subsystem))#slow corla
        self.gadget_controller.rightTrigger().whileTrue(intake.fastForwardCoralCommand(self.intake_subsystem))#fast coral

        self.timer = Timer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    
    def autonomousInit(self) -> None:
        # commands2.CommandScheduler.getInstance().schedule(commands2.InstantCommand(lambda: self.swerve.drive(-0.2, 0, 0, False, True)))
        # self.timer = Timer()
        # self.timer.start()

        # if self.timer.get() < 2.0:
        #     self.swerve.drive(0.2, 0, 0, False, True)
        # else:
        #     self.swerve.drive(0, 0, 0, False, True)
        self.timer.reset()
        self.timer.start()


    def autonomousPeriodic(self) -> None: 
        if self.timer.get() < 1.75:
            self.swerve.drive(0, 0, .55, False, True)
        elif self.timer.get() >= 1.75 and self.timer.get() < 4.25:
            self.swerve.drive(0.2, 0, 0, False, True)
        elif self.timer.get() >= 4.25 and self.timer.get() < 6:
            self.swerve.drive(0, 0, 0, False, True)
            self.elevator_subsystem.l2()
            if self.timer.get() >= 5.25 and self.timer.get() < 6:
                self.intake_subsystem.slowForwardCoral()
        else:
            self.swerve.drive(0, 0, 0, False, True)
            self.elevator_subsystem.stop()
            self.intake_subsystem.stop()

    def teleopInit(self) -> None:
        pass
    

    def teleopPeriodic(self) -> None:
        # Teleop periodic logic
        if self.driver_controller.getLeftTriggerAxis() > 0.1:
            self.slowdwj(False)
        else:
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

    def slowdwj(self, field_relative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        x_speed = (
            -self.x_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftY(), 0.08)
            )
             * 0.2
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        y_speed = (
            -self.y_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftX(), 0.08)
            )
             * 0.2
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rot_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getRightX(), 0.08)
            )
             * 0.2
        )


        self.swerve.drive(x_speed, y_speed, rot,field_relative, rate_limit=True)

if __name__ == "__main__":
    wpilib.run(MyRobot)