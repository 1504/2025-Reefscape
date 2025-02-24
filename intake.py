import rev
import wpilib
from wpilib import TimedRobot, Joystick
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
import constants

class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #0 & 1 are placeholder numbers
        self.leftMotor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
        self.rightMotor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)#inverted

        #config variables
        self.leftMotorConfig = SparkMaxConfig()
        self.rightMotorConfig = SparkMaxConfig()

        #encoders
        self.intakeEncoder1 = self.leftMotor.getEncoder()
        self.intakeEncoder2 = self.rightMotor.getEncoder()

        #pid controllers 
        self.pidCE1 = self.leftMotor.getClosedLoopController()
        self.pidCE2 = self.rightMotor.getClosedLoopController()
        self.leftMotorConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.rightMotorConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        
        # Initial gains
        self.leftMotorConfig.closedLoop.P(0.1)
        self.leftMotorConfig.closedLoop.I(0.01)
        self.leftMotorConfig.closedLoop.D(0.001)
        #self.leftMotorConfig.closedLoop.velocityFF(0)
        #self.leftMotorConfig.closedLoop.outputRange(0,0)

        # Initial gains
        self.rightMotorConfig.closedLoop.P(0.1)
        self.rightMotorConfig.closedLoop.I(0.01)
        self.rightMotorConfig.closedLoop.D(0.001)
        #self.rightMotorConfig.closedLoop.velocityFF(0)
        #self.rightMotorConfig.closedLoop.outputRange(0,0)

    def intake(self):
        current_position = self.intakeEncoder1.get()
        control_effort = self.pidCE1.setReference(current_position, 1)
        self.leftMotor.set(control_effort)
        self.rightMotor.set(control_effort)
        #placeholder values
        #self.leftMotor.set(0.8)
        #self.rightMotor.set(-0.8)
    
    def stop(self):
        self.rightMotor.set(0)
        self.leftMotor.set(0)


class IntakeCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.intake()

    def end(self, interrupted):
        self.intake_subsystem.stop()

   