import rev
import wpilib
from wpilib import TimedRobot, Joystick
import commands2
from commands2 import Command
from commands2 import Subsystem

class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #0 & 1 are placeholder numbers
        self.leftMotor = rev.SparkMax(9, rev.SparkMax.MotorType.kBrushless)#inverted
        self.rightMotor = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)
        self.motors = wpilib.MotorControllerGroup(self.leftMotor, self.rightMotor)

    def intake(self):
        #placeholder values
        self.leftMotor.set(-1.0)
        self.rightMotor.set(1.0)
    
    def stop(self):
        self.leftMotor.set(0.0)
        self.rightMotor.set(0.0)


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

   