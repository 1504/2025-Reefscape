import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
import constants



class AlgaeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.greenWheelMotor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)
        self.jointMotor = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)



        #self.intake_complete = False
        #self.timer = wpilib.Timer()

    def outwardClaw(self):
        self.jointMotor.set(.04) # Deploys the claw at a slow speed
    
    def inwardClaw(self):
        self.jointMotor.set(-.04) #1 for first place
    
    def pauseClaw(self):
        self.jointMotor.set(0)

    def turnWheelFast(self):
        self.greenWheelMotor.set(-0.075)

    def turnWheelSlow(self):
        self.greenWheelMotor.set(-0.005)

    def stopWheels(self):
        self.greenWheelMotor.set(0.0)
    

    
    

class outwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        self.algae_subsystem.outwardClaw()

    def execute(self):
        pass
    def isFinished(self):
        return True
    
    def end(self, interrupted):
        pass

class inwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        self.algae_subsystem.inwardClaw()
    def isFinished(self):
        return True
        
    def execute(self):
        pass
            
    def end(self, interrupted):
        pass


class holdAlgaeCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        self.algae_subsystem.turnWheelFast()

    def execute(self):
        pass
    def isFinished(self):
        return True

    def end(self, interrupted):
        pass


class dropAlgaeCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem   

    def initialize(self):
        self.algae_subsystem.stopWheels()    

    def execute(self):
        pass
    def isFinished(self):
        return True
    def end(self, interrupted):
        pass

