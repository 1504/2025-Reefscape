import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import navx
import drivesubsystem
import commands2
import elevator
import limelightCamera
import constants
import intake

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
        self.camera = limelightCamera.LimelightCamera("Limelightt")
        

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)

        #self.driver_controller.a().whileTrue(commands2.InstantCommand(lambda: self.swerve.drive(0,0,self.camera.getX()* -0.005,False, rate_limit=True)))
        #self.driver_controller.a().onFalse(commands2.InstantCommand(lambda: self.swerve.drive(0, 0, 0, False, False)))
       
    def robotPeriodic(self):
        print()
    
    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass
    

    def teleopPeriodic(self) -> None:
        # Teleop periodic logic
        # we don't want the robot to accidently drive during this testing.
        self.getXofObject()


    def testPeriodic(self) -> None:
        pass

    def getXofObject(self) -> None:
        targetX = self.camera.getX()
        print(targetX)
        turn_speed = -0.005 * targetX
        self.swerve.drive(0,0,turn_speed,True,rate_limit=True)

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