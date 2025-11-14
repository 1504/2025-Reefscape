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
import ntcore

# To see messages from networktables:
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
        
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)

        # Algae Bindings
        self.gadget_controller.povRight().onTrue(algae.inwardClawCommand(self.algae_subsystem))
        self.gadget_controller.povUp().toggleOnFalse(algae.holdAlgaeCommand(self.algae_subsystem))
        self.gadget_controller.povLeft().onTrue(algae.outwardClawCommand(self.algae_subsystem))

        # elevator bindings
        self.gadget_controller.a().whileTrue(elevator.ElevatorDownCommand(self.elevator_subsystem))
        self.gadget_controller.y().whileTrue(elevator.ElevatorL4Command(self.elevator_subsystem))
        self.gadget_controller.x().whileTrue(elevator.ElevatorL3Command(self.elevator_subsystem))
        self.gadget_controller.b().whileTrue(elevator.ElevatorL2Command(self.elevator_subsystem))
        commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() < -0.5).whileTrue(elevator.UpCommand(self.elevator_subsystem))
        commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() > 0.5).whileTrue(elevator.ElevatorDownManualCommand(self.elevator_subsystem))

        # coral Bindings
        self.gadget_controller.leftBumper().whileTrue(intake.PrimeCoralCommand(self.intake_subsystem))
        self.gadget_controller.leftTrigger().whileTrue(intake.BackCoralCommand(self.intake_subsystem))
        self.gadget_controller.rightBumper().whileTrue(intake.slowForwardCoralCommand(self.intake_subsystem))#slow corel
        self.gadget_controller.rightTrigger().whileTrue(intake.fastForwardCoralCommand(self.intake_subsystem))#fast coral


        ## Core Functions
        ## Core Functions


        # NetworkTable initialization

        self.coreTableInstance = ntcore.NetworkTableInstance.getDefault()


        # Limelight subscribing and publishing setup

        self.limelightTable = self.coreTableInstance.getTable("limelight") # IP's not work here



        self.txLimelightSub = self.limelightTable.getDoubleTopic("tx").subscribe(0.0) # via limelight to limelightTable, for direct on robot retrieval

        self.tyLimelightSub = self.limelightTable.getDoubleTopic("ty").subscribe(0.0)

        self.taLimelightSub = self.limelightTable.getDoubleTopic("ta").subscribe(0.0)

        self.tvLimelightSub = self.limelightTable.getDoubleTopic("tv").subscribe(0)

        self.tidLimelightSub = self.limelightTable.getDoubleTopic("tid").subscribe(0.0)




        # dataArray from 3D AprilTag

        self.dataArrayLimelightSub = self.limelightTable.getDoubleArrayTopic("camerapose_targetspace").subscribe([6])



        # Changing settings on Limelight

        self.pipelineLimelightPub = self.limelightTable.getDoubleTopic("pipeline").publish() # via limelightTable to limelight

        self.pipelineLimelightPub.set(limelight1_DefaultPipeline) # Default pipeline

        self.streamLimelightPub = self.limelightTable.getDoubleTopic("stream").publish()

        self.coreTable = self.coreTableInstance.getTable("datatable")


        # General subscribing and publishing setup

        # Generic part 1 code for declaring a telemetry publishing ( this declaration is placed in robotInit)

        # self.xxxPub = self.coreTable.getDoubleTopic("xxx").publish()

        # self.xxxPub = self.coreTable.getBooleanTopic("xxx").publish()


        self.txPub = self.coreTable.getDoubleTopic("tx").publish() # via coreTable to shuffleboard

        self.tyPub = self.coreTable.getDoubleTopic("ty").publish()

        self.taPub = self.coreTable.getDoubleTopic("ta").publish()

        self.tvPub = self.coreTable.getDoubleTopic("tv").publish()

        self.tidPub = self.coreTable.getDoubleTopic("tid").publish()

        self.pipelinePub = self.coreTable.getDoubleTopic("pipeline").publish()



        # Publishing elements form dataArray

        self.txFromDataArrayPub = self.coreTable.getDoubleTopic("txFromDataArray").publish()

        self.tzFromDataArrayPub = self.coreTable.getDoubleTopic("tzFromDataArray").publish()

        self.RyFromDataArrayPub = self.coreTable.getDoubleTopic("RyFromDataArray").publish()

        self.targetDistancePub = self.coreTable.getDoubleTopic("targetDistance").publish()



        # Drive system telemetry

        self.driveFR_EncPub = self.coreTable.getDoubleTopic("driveFR_Enc").publish()

        self.driveFL_EncPub = self.coreTable.getDoubleTopic("driveFL_Enc").publish()

        self.driveBL_EncPub = self.coreTable.getDoubleTopic("driveBL_Enc").publish()

        self.driveBR_EncPub = self.coreTable.getDoubleTopic("driveBR_Enc").publish()


        self.rotationFR_EncPub = self.coreTable.getDoubleTopic("rotationFR_Enc").publish()

        self.rotationFL_EncPub = self.coreTable.getDoubleTopic("rotationFL_Enc").publish()

        self.rotationBR_EncPub = self.coreTable.getDoubleTopic("rotationBR_Enc").publish()

        self.rotationBL_EncPub = self.coreTable.getDoubleTopic("rotationBL_Enc").publish()



        self.timer = Timer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    
    def autonomousInit(self) -> None:
        self.timer.reset()
        self.timer.start()
        self.auton_timer=1.9


    def autonomousPeriodic(self) -> None:
        if self.timer.get() < self.auton_timer:
            self.swerve.drive(0, 0, .55, False, True)
        elif self.timer.get() >= self.auton_timer and self.timer.get() < self.auton_timer+3:
            self.swerve.drive(0.2, 0, 0, False, True)
        elif self.timer.get() >= self.auton_timer+3 and self.timer.get() < self.auton_timer+5.05:
            self.swerve.drive(0, 0, 0, False, True)
            if self.timer.get() >= self.auton_timer+4.75 and self.timer.get() < self.auton_timer+5.05:
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
        elif self.driver_controller.getRightTriggerAxis() > 0.1:
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
        x_speed = (
            -self.x_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftY(), 0.08)
            )
             * 0.2
        )

        y_speed = (
            -self.y_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftX(), 0.08)
            )
             * 0.2
        )

        rot = (
            -self.rot_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getRightX(), 0.08)
            )
             * 0.2
        )


        self.swerve.drive(x_speed, y_speed, rot,field_relative, rate_limit=True)

if __name__ == "__main__":
    wpilib.run(MyRobot)