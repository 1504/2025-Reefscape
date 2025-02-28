import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput
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
        self.rightMotor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)
        #self.placeholderNumber = 0 #Replace with actual number
        self.coralSensor = DigitalInput(9)
        #conveyorDetector.whileActiveContinuous(new RunMotor());
        #self.motors = wpilib.MotorControllerGroup(self.leftMotor, self.rightMotor)

    def intake(self):
        #current_position = self.intakeEncoder1.get()
        #control_effort = self.pidCE1.setReference(current_position, 1)
        #self.leftMotor.set(control_effort)
        #self.rightMotor.set(control_effort)
        #placeholder values
        self.leftMotor.set(0.8)
        self.rightMotor.set(-0.8)
    
    def stop(self):
        self.leftMotor.set(0.0)
        self.rightMotor.set(0.0)
    
    #def getCoralSensorInput(self):
        #self.coralSensor.get()#true when & false when?
    
    def primeCoral(self):
        self.leftMotor.set(.5)
        self.rightMotor.set(-.5)
        self.i = 0
        if self.coralSensor.get():
            self.leftMotor.set(.5)
            self.rightMotor.set(-.5)
            i = 1
        if self.coralSensor.get() == False and i == 1:
            self.leftMotor.set(0.0)
            self.rightMotor.set(0.0)
            i = 0
        #motors stop
    
    def releaseCoral(self):
        #button press and go
        self.leftMotor.set(0.8)
        self.rightMotor.set(-0.8)


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

class PrimeCoralCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.primeCoral()

    def end(self, interrupted):
        self.intake_subsystem.stop()

class ReleaseCoralCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.releaseCoral()

    def end(self, interrupted):
        self.intake_subsystem.stop()

   