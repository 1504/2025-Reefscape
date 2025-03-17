import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
import constants

#sorryyyy i can't think of a better name

class AlgaeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.greenWheelMotor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)
        self.jointMotor = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)

        #self.intake_complete = False
        #self.timer = wpilib.Timer()

    def outwardClaw(self):
        self.jointMotor.set(.05) # Deploys the claw at a slow speed
    
    def inwardClaw(self):
        self.jointMotor.set(-.05) #1 for first place
    
    def pauseClaw(self):
        self.jointMotor.set(0)

    def turnWheelFast(self):
        self.greenWheelMotor.set(-0.1)

    def turnWheelSlow(self):
        self.greenWheelMotor.set(-0.05)


    
    def stopWheels(self):
        self.greenWheelMotor.set(0.0)
    

    
    

class outwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.algae_subsystem.outwardClaw()

    def end(self, interrupted):
        self.algae_subsystem.pauseClaw()

class inwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.algae_subsystem.inwardClaw()

    def end(self, interrupted):
        self.algae_subsystem.pauseClaw()


class grabAlgaeCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.algae_subsystem.turnWheelFast()

    def end(self, interrupted):
        self.algae_subsystem.stopWheels()

class holdAlgaeCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.algae_subsystem.turnWheelSlow()

    def end(self, interrupted):
        self.algae_subsystem.stopWheels()
class dropalgaecommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem   

    def initialize(self):
            pass

    def execute(self):
        self.algae_subsystem.stopWheels()

