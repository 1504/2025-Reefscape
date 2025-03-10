import math

import wpilib
import wpimath.geometry
import wpimath.kinematics
import wpimath.filter
import wpimath.units
import ntcore
import navx

import swervemodule
import constants
import swerveutils

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
#from wpilib import DriverStation

from commands2 import Command
# import networklogger

class DriveSubsystem:
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self) -> None:
        self.kDriveKinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            wpimath.geometry.Translation2d(constants.kWheelBase / 2, constants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(constants.kWheelBase / 2, -constants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(-constants.kWheelBase / 2, constants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(-constants.kWheelBase / 2, -constants.kTrackWidth / 2),
        )

        self.front_left = swervemodule.SwerveModule(
            constants.kFrontLeftDrivingCanId, 
            constants.kFrontLeftTurningCanId,
            constants.kFrontLeftChassisAngularOffset,
            constants.kFrontLeftAbsoluteEncoderOffset
        )
        self.rear_left = swervemodule.SwerveModule(
            constants.kRearLeftDrivingCanId,
            constants.kRearLeftTurningCanId,
            constants.kRearLeftChassisAngularOffset,
            constants.kRearLeftAbsoluteEncoderOffset
        )
        self.front_right = swervemodule.SwerveModule(
            constants.kFrontRightDrivingCanId,
            constants.kFrontRightTurningCanId,
            constants.kFrontRightChassisAngularOffset,
            constants.kFrontRightAbsoluteEncoderOffset
        )
        self.rear_right = swervemodule.SwerveModule(
            constants.kRearRightDrivingCanId,
            constants.kRearRightTurningCanId,
            constants.kRearRightChassisAngularOffset,
            constants.kRearRightAbsoluteEncoderOffset
        )

        self.swerve_modules = [self.front_left, self.rear_left, self.front_right, self.rear_right]

        # the gyro sensor
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        # Slew rate filter variables for controlling the lateral acceleration
        self.current_rotation = 0.0
        self.current_translation_dir = 0.0
        self.current_translation_mag = 0.0

        self.mag_limiter = wpimath.filter.SlewRateLimiter(constants.kMagnitudeSlewRate / 1)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(constants.kRotationalSlewRate / 1)

        self.prev_time = ntcore._now() * pow(1, -6) # secodns

        # Odometry class for tracking robot pose
        # 4 defines the number of modules
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kDriveKinematics,
            # wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.getAngle())),
            self.gyro.getRotation2d(),
            (self.front_left.get_position(), self.front_right.get_position(), self.rear_left.get_position(), self.rear_right.get_position()),
            wpimath.geometry.Pose2d()
        )

        self.reset_encoders()

        # logger object for sending data to smart dashboard
        # self.logger = networklogger.NetworkLogger()

        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        robot_config = RobotConfig.fromGUISettings()

        # # Configure the AutoBuilder last

        # AutoBuilder.configure(
        #     self.getPose, # Robot pose supplier
        #     self.resetOdometry, # Method to reset odometry (will be called if your auto has a starting pose)
        #     self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        #     lambda speeds, feedforwards: self.drive(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
        #     PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
        #         PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
        #         PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
        #     ),
        #     config, # The robot configuration
        #     self.shouldFlipPath, # Supplier to control path flipping based on alliance color
        #     self # Reference to this subsystem to set requirements
        # )

        holonomic_controller = PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
            )

        pose_supplier=self.getPose()
        self.resetOdometry(pose_supplier) #needs parameters
        #robot_relative_speeds_supplier=self.getRobotRelativeSpeeds()
        # output=self.driveRobotRelative()
        # controller=holonomic_controller
        # robot_config=robot_config
        # should_flip_path=self.shouldFlipPath()
        # drive_subsystem=self
        
        # AutoBuilder.configure(
        #         pose_supplier=self.getPose,
        #         reset_pose=self.resetOdometry,
        #         robot_relative_speeds_supplier=self.getRobotRelativeSpeeds,
        #         output=self.driveRobotRelative, 
        #         controller=holonomic_controller,
        #         robot_config=robot_config,
        #         should_flip_path=self.shouldFlipPath,
        #         drive_subsystem=self
        # )

        self.automated_path = None #is this necessary??

    def periodic(self):
        self.odometry.update(
            # wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.getAngle())),
            self.gyro.getRotation2d(),
            (self.front_left.get_position(), self.front_right.get_position(), self.rear_left.get_position(), self.rear_right.get_position()),
        )

        #self.logger.log_gyro(self.gyro.getAngle())



    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
        rate_limit: bool,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param x_speed: Speed of the robot in the x direction (forward).
        :param y_speed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param field_relative: Whether the provided x and y speeds are relative to the field.
        :param rate_limit: Whether to enable rate limiting for smoother control
        :param periodSeconds: Time
        """
        x_speed_commanded = None
        y_speed_commanded = None

        if rate_limit:
            # Convert XY to polar for rate limiting
            input_translation_dir = math.atan2(y_speed, x_speed)
            input_translation_mag = math.sqrt(pow(x_speed, 2) + pow(y_speed, 2))

            # Calculate the direction slew rate based on an estimate of lateral acceleration
            direction_slew_rate = None
            if self.current_translation_mag != 0.0:
                direction_slew_rate = abs(constants.kDirectionSlewRate / self.current_translation_mag)
            else:
                direction_slew_rate = 500.0 # some high number that means the slew rate is effectively instantaneous
            
            current_time = ntcore._now() * pow(1, -6)
            elapsed_time = current_time - self.prev_time
            angleDif = swerveutils.angleDifference(input_translation_dir, self.current_translation_dir)

            if angleDif < 0.45 * math.pi:
                self.current_translation_dir = swerveutils.stepTowardsCircular(self.current_translation_dir, input_translation_dir, direction_slew_rate * elapsed_time)
                self.current_translation_mag = self.mag_limiter.calculate(input_translation_mag)
            elif angleDif > 0.85 * math.pi:
                if self.current_translation_mag > 1e-4:
                    self.current_translation_mag = self.mag_limiter.calculate(0.0)
                else:
                    self.current_translation_dir = swerveutils.wrapAngle(self.current_translation_dir + math.pi)
                    self.current_translation_mag = self.mag_limiter.calculate(input_translation_mag)
            else:
                self.current_translation_dir = swerveutils.stepTowardsCircular(self.current_translation_dir, input_translation_dir, direction_slew_rate * elapsed_time)
                self.current_translation_mag = self.mag_limiter.calculate(0.0)
            
            self.prev_time = current_time

            x_speed_commanded = self.current_translation_mag * math.cos(self.current_translation_dir)
            y_speed_commanded = self.current_translation_mag * math.sin(self.current_translation_dir)
            self.current_rotation = self.rot_limiter.calculate(rot)
        else:
            x_speed_commanded = x_speed
            y_speed_commanded = y_speed
            self.current_rotation = rot
        
        # Convert the commanded speeds into correct units for the drivetrain
        x_speedDelivered = x_speed_commanded * constants.kMaxSpeed
        y_speedDelivered = y_speed_commanded * constants.kMaxSpeed
        rotDelivered = self.current_rotation * constants.kMaxAngularSpeed

        (fl, fr, bl, br) = self.kDriveKinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speedDelivered, y_speedDelivered, rotDelivered, self.gyro.getRotation2d()#wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.getAngle()))
            ) if field_relative 
            else wpimath.kinematics.ChassisSpeeds(x_speedDelivered, y_speedDelivered, rotDelivered)
        )


        # Set the swerve modules to desired states
        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.rear_left.set_desired_state(bl)
        self.rear_right.set_desired_state(br)



    def setX(self) -> None:
        self.front_left.set_desired_state(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(45))))
        self.front_right.set_desired_state(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(-45))))
        self.rear_left.set_desired_state(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(-45))))
        self.rear_right.set_desired_state(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(45))))

    def setModuleStates(self, desiredStates: tuple[wpimath.kinematics.SwerveModuleState]) -> None:
        self.kDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.kMaxSpeed)

        self.front_left.set_desired_state(desiredStates[0])
        self.front_right.set_desired_state(desiredStates[1])
        self.rear_left.set_desired_state(desiredStates[2])
        self.rear_right.set_desired_state(desiredStates[3])

    def reset_encoders(self) -> None:
        self.front_left.reset_encoders()
        self.rear_left.reset_encoders()
        self.front_right.reset_encoders()
        self.rear_right.reset_encoders()

    # Returns the robot's heading in degrees from -180 to 180
    def getHeading(self) -> float:
        return self.gyro.getRotation2d().degrees()
        # return wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.getAngle())).degrees()

    # Zeroes the heading of the robot
    def zeroHeading(self) -> None:
        self.gyro.reset()

    # Returns the turn rate of the robot in degrees per second
    def getTurnRate(self) -> float:
        return -self.gyro.getRate()

    # returns the currently-estimated pose
    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.odometry.getPose()
    
    # Resets the odometry to the specified pose
    def resetOdometry(self, pose: wpimath.geometry.Pose2d):
        #self.odometry.resetPosition(self.getHeading(), self.get_module_positions(), pose)
        self.odometry.resetPosition(50, (3,2,1,4), pose)
        #self.odometry.resetPosition(self.getHeading(), (self.front_left.get_position(), self.front_right.get_position(), self.rear_left.get_position(), self.rear_right.get_position()), pose)

    def get_module_positions(self):
        return [m.get_position() for m in self.swerve_modules]
    
    # #EXTRA STUFF FOR PATHPLANNER
        
    def getRobotRelativeSpeeds(self):
        return wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
            self.front_left.get_state(self), self.rear_left.get_state(self), self.front_right.get_state(self), self.rear_right.get_state(self))

    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            return False
        else:
            return True
    
    # #do we need a driveRobotRelative or do we already have that
    def driveRobotRelative(self, chassis_speeds: wpimath.kinematics.ChassisSpeeds, feedforwards, desiredStates: tuple[wpimath.kinematics.SwerveModuleState]):
        # required for the pathplanner lib's pathfollowing based on chassis speeds
        # idk if we need the feedforwards
        # swerveModuleStates = self.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
        swerveModuleStates = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, constants.kMaxSpeed)
        #for state, module in zip(swerveModuleStates, self.swerve_modules):
            #module.set_desired_state(state)
        
        self.front_left.set_desired_state(desiredStates[0])
        self.front_right.set_desired_state(desiredStates[1])
        self.rear_left.set_desired_state(desiredStates[2])
        self.rear_right.set_desired_state(desiredStates[3])