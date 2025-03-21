import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
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
        self.coralSensor2 = DigitalInput(8)
        self.intake_complete = False
        self.timer = wpilib.Timer()

        #conveyorDetector.whileActiveContinuous(new RunMotor());
        #self.motors = wpilib.MotorControllerGroup(self.leftMotor, self.rightMotor)

    def fastForwardCoral(self):
        #current_position = self.intakeEncoder1.get()
        #control_effort = self.pidCE1.setReference(current_position, 1)
        #self.leftMotor.set(control_effort)
        #self.rightMotor.set(control_effort)
        #placeholder values
        self.leftMotor.set(-0.6)
        self.rightMotor.set(0.6)

    def backCoral(self):
        #current_position = self.intakeEncoder1.get()
        #control_effort = self.pidCE1.setReference(current_position, 1)
        #self.leftMotor.set(control_effort)
        #self.rightMotor.set(control_effort)
        #placeholder values
        self.leftMotor.set(0.1)
        self.rightMotor.set(-0.1)
    
    def stop(self):
        self.leftMotor.set(0.0)
        self.rightMotor.set(0.0)
    
    #def getCoralSensorInput(self):
        #self.coralSensor.get()#true when & false when?
    
    def primeCoral(self):
        if self.coralSensor.get() and self.coralSensor2.get():
            self.leftMotor.set(-.2)
            self.rightMotor.set(.2)
            print(self.coralSensor.get())
            print(self.coralSensor2.get())
        elif self.coralSensor.get() == False and self.coralSensor2.get() == False:
            print ('slowing')
            self.leftMotor.set(-.05)
            self.rightMotor.set(.05)
        elif self.coralSensor.get() == False and self.coralSensor2.get():
            print ("stopping")
            self.leftMotor.set(0.0)
            self.rightMotor.set(0.0)
            return True
        return False

    
    
    def slowForwardCoral(self):
        #button press and go
        self.leftMotor.set(-0.2)
        self.rightMotor.set(0.2)


class fastForwardCoralCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.fastForwardCoral()

    def end(self, interrupted):
        self.intake_subsystem.stop()

class PrimeCoralCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem
        self.complete = False

    def initialize(self):
        pass

    def execute(self):
        self.complete = self.intake_subsystem.primeCoral()

    def isFinished(self):
        return self.complete

    def end(self, interrupted):
        self.intake_subsystem.stop()

class slowForwardCoralCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.slowForwardCoral()

    def end(self, interrupted):
        self.intake_subsystem.stop()

class BackCoralCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.backCoral()

    def end(self, interrupted):
        self.intake_subsystem.stop()

class STOPCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.stop()

    def end(self, interrupted):
        self.intake_subsystem.stop()


   