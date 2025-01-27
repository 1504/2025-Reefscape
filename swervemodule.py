import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
import constants

kWheelRadius = 0.0508
kEncoderResolution = 4096
#rev neo is 42 for encoder resolutionz: try this out
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, chassisAngularOffset: float) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        """

        """ Initialize Spark Max motor controllers"""
        self.drivingSparkMax: SparkMax = SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless)
        self.turningSparkMax: SparkMax = SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless)

        """ Create Config Variables """ 
        self.driving_config = SparkMaxConfig()
        self.turning_config = SparkMaxConfig()

        """ Initialize Spark Max encoders"""
        # Get Encoder Objects from Spark Max
        # driving: relative encoder
        # turning: absolute encoder
        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.turningEncoder = self.turningSparkMax.getAbsoluteEncoder()

        # Apply position and velocity conversion factors for the driving encoder.
        # We want these in radians and radians per second to use with WPILibs swerve APIs
        self.driving_config.encoder.positionConversionFactor(constants.kDrivingEncoderPositionFactor)
        self.driving_config.encoder.velocityConversionFactor(constants.kDrivingEncoderVelocityFactor)

        # Apply position and velocity conversion factors for the turning encoder.
        # We want these in radians and radians per second to use with WPILibs swerve APIs
        self.turning_config.encoder.positionConversionFactor(constants.kTurningEncoderPositionFactor)
        self.turning_config.encoder.velocityConversionFactor(constants.kTurningEncoderVelocityFactor)

        # Invert the turning encoder, since the output shaft rotates in the opposite
        # direction of the steering motor in the MAXSwerve Module.
        # self.turningEncoder.setInverted(constants.kTurningEncoderInverted)
        # self.turning_config.encoder(constants.kTurningEncoderInverted)
        self.turning_config.absoluteEncoder.inverted(constants.kTurningEncoderInverted)

        """ Initialize PID Controllers"""
        # create spark max pid controllers
        # self.drivingPIDController: rev.SparkPIDController = self.drivingSparkMax.getPIDController()
        # self.turningPIDController: rev.SparkPIDController = self.turningSparkMax.getPIDController()
        self.drivingPIDController = self.drivingSparkMax.getClosedLoopController()
        self.turningPIDController = self.turningSparkMax.getClosedLoopController()

        self.driving_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.turning_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)


        # Enable PID wrap around for the turning motor. This will allow the PID
        # controller to go through 0 to get to the setpoint i.e. going from 350
        # degrees to 10 degrees will go through 0 rather than the other direction
        #  which is a longer route.
        self.turning_config.closedLoop.positionWrappingEnabled(True)
        self.turning_config.closedLoop.positionWrappingMinInput(constants.kTurningEncoderPositionPIDMinInput)
        self.turning_config.closedLoop.positionWrappingMaxInput(constants.kTurningEncoderPositionPIDMaxInput)
        
        
        # Set the PID gains for the driving motor. Note these are example gains, and
        # you may need to tune them for your own robot!
        self.driving_config.closedLoop.P(constants.kDrivingP)
        self.driving_config.closedLoop.I(constants.kDrivingI)
        self.driving_config.closedLoop.D(constants.kDrivingD)
        self.driving_config.closedLoop.velocityFF(constants.kDrivingFF)
        self.driving_config.closedLoop.outputRange(constants.kDrivingMinOutput, constants.kDrivingMaxOutput)

        # Set the PID gains for the turning motor. Note these are example gains, and
        # you may need to tune them for your own robot!
        self.turning_config.closedLoop.P(constants.kTurningP)
        self.turning_config.closedLoop.I(constants.kTurningI)
        self.turning_config.closedLoop.D(constants.kTurningD)
        self.turning_config.closedLoop.velocityFF(constants.kTurningFF)
        self.turning_config.closedLoop.outputRange(constants.kTurningMinOutput, constants.kTurningMaxOutput)

        """ Spark Max Mode Parameters"""
        self.driving_config.setIdleMode(constants.kDrivingMotorIdleMode)
        self.turning_config.setIdleMode(constants.kTurningMotorIdleMode)
        self.driving_config.smartCurrentLimit(constants.kDrivingMotorCurrentLimit)
        self.turning_config.smartCurrentLimit(constants.kTurningMotorCurrentLimit)

        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations
        # self.drivingSparkMax.burnFlash()
        # self.turningSparkMax.burnFlash()

        self.drivingSparkMax.configure(self.driving_config,
                                          SparkBase.ResetMode.kResetSafeParameters,
                                          SparkBase.PersistMode.kPersistParameters)
        self.turningSparkMax.configure(self.turning_config,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        # Swerve drive parameters
        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = wpimath.kinematics.SwerveModuleState(0.0, wpimath.geometry.Rotation2d(self.turningEncoder.getPosition()))
        self.drivingEncoder.setPosition(0)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.drivingEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.drivingEncoder.getPosition(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        # Apply chassis angular offset to the desired state
        correctedDesiredState = wpimath.kinematics.SwerveModuleState(desiredState.speed, desiredState.angle + wpimath.geometry.Rotation2d(self.chassisAngularOffset))

        turningEncoderPosition = wpimath.geometry.Rotation2d(self.turningEncoder.getPosition())
        # Optimize the reference state to avoid spinning further than 90 degrees
        optimizedDesiredState = wpimath.kinematics.SwerveModuleState.optimize(
            correctedDesiredState, turningEncoderPosition
        )

        # Command driving and turning SPARK MAX toward their respective setpoints
        self.drivingPIDController.setReference(optimizedDesiredState.speed, rev.CANSparkMax.ControlType.kVelocity)
        self.turningPIDController.setReference(optimizedDesiredState.angle.radians(), rev.CANSparkMax.ControlType.kPosition)

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        self.drivingEncoder.setPosition(0)