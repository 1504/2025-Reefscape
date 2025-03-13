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

        self.jointMotor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)
        self.clawMotor = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)

        #self.intake_complete = False
        #self.timer = wpilib.Timer()

    def outwardClaw(self):
        self.clawMotor.set(.05) #1 for first place
    
    def inwardClaw(self):
        self.clawMotor.set(-.05) #1 for first place

    def swivelTheJoint(self):
        self.jointMotor.set(-0.1)
        pass #ask build team whether they want it on a joystick or buttons
    
    def stopJointMotor(self):
        self.jointMotor.set(0.0)
    
    def stopClawMotor(self):
        self.clawMotor.set(0.0)
    
    

class outwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.algae_subsystem.outwardClaw()

    def end(self, interrupted):
        self.algae_subsystem.stopClawMotor()

class inwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.algae_subsystem.inwardClaw()

    def end(self, interrupted):
        self.algae_subsystem.stopClawMotor()
class jointrotatecommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        pass

    def execute(self):
        self.algae_subsystem.swivelTheJoint()

    def end(self, interrupted):
        self.algae_subsystem.stopJointMotor()



