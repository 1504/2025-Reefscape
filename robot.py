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
from wpilib import Timer

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.driver_controller = commands2.button.CommandXboxController(0)
        self.gadget_controller = commands2.button.CommandXboxController(1)
        self.swerve = drivesubsystem.DriveSubsystem()
        self.elevator_subsystem = elevator.ElevatorSubsystem()
        self.intake_subsystem = intake.IntakeSubsystem()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)
        
        #elevator
        self.gadget_controller.a().whileTrue(elevator.ElevatorDownCommand(self.elevator_subsystem)) #south
        self.gadget_controller.b().whileTrue(elevator.ElevatorL2Command(self.elevator_subsystem)) #east
        self.gadget_controller.x().whileTrue(elevator.ElevatorL3Command(self.elevator_subsystem)) #west
        self.gadget_controller.y().whileTrue(elevator.ElevatorL4Command(self.elevator_subsystem)) #north
        
        #intake 
        self.gadget_controller.leftBumper().whileTrue(intake.PrimeCoralCommand(self.intake_subsystem)) #priming, fix
        self.gadget_controller.leftTrigger().whileTrue(intake.BackCoralCommand(self.intake_subsystem)) # adjustment backwards
        self.gadget_controller.rightBumper().whileTrue(intake.slowForwardCoralCommand(self.intake_subsystem))#slow coral score
        self.gadget_controller.rightTrigger().whileTrue(intake.fastForwardCoralCommand(self.intake_subsystem))#fast coral score

        #for small asjustments - temporary solution for the levels, hopefullly we can score by just holding the l# buttons.
        self.gadget_controller.povUp().whileTrue(elevator.ElevatorUpAdjustmentCommand(self.elevator_subsystem))
        self.gadget_controller.povDown().whileTrue(elevator.ElevatorDownAdjustmentCommand(self.elevator_subsystem))
    
    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        commands2.CommandScheduler.registerSubsystem(self.elevator_subsystem)
        commands2.CommandScheduler.registerSubsystem(self.intake_subsystem)
    
    def autonomousInit(self) -> None:
        #temporary soloution to auton, fix on monday
        commands2.CommandScheduler.getInstance().schedule(commands2.InstantCommand(lambda: self.swerve.drive(-0.2, 0, 0, False, True)))
        

    def autonomousPeriodic(self) -> None: 
        pass

    def teleopInit(self) -> None:
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

if __name__ == "__main__":
    wpilib.run(MyRobot)