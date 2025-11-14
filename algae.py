import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
import constants

import time

class AlgaeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.greenWheelMotor: SparkMax = SparkMax(13, SparkMax.MotorType.kBrushless)
        self.jointMotor: SparkMax = SparkMax(14, SparkMax.MotorType.kBrushless)

    def outwardClaw(self):
        self.jointMotor.set(0.2) # Deploys the claw at a slow speed
    
    def inwardClaw(self):
        self.jointMotor.set(-0.15) #1 for first place

    def turnWheelFast(self):
        self.greenWheelMotor.set(-0.1)

    def stop(self):
        self.greenWheelMotor.set(0.0)
        self.jointMotor.set(0.0)
    

    
    

class outwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        self.algae_subsystem.outwardClaw()
        self.start_time = time.time()
        self.inTime = time.time() + 1.37

    def execute(self):
        pass

    def isFinished(self):
        return time.time() > self.inTime

    def end(self, interrupted):
        self.algae_subsystem.stop()

class inwardClawCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        self.algae_subsystem.inwardClaw()
        self.start_time = time.time()
        self.inTime = time.time() + 1.37  # Hold for 1 seconds
        
    def execute(self):
        pass

    def isFinished(self):
        return time.time() > self.inTime
            
    def end(self, interrupted): 
        self.algae_subsystem.stop()


class holdAlgaeCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem

    def initialize(self):
        self.algae_subsystem.turnWheelFast()
           
    def isFinished(self):
        pass
    
    def execute(self):
        pass

    def end(self, interrupted):
        self.algae_subsystem.stop()


class dropAlgaeCommand(Command):
    def __init__(self, algae_subsystem):
        super().__init__()

        self.algae_subsystem = algae_subsystem   

    def initialize(self):
        self.algae_subsystem.stop()    

    def execute(self):
        pass
    def end(self, interrupted):
        self.algae_subsystem.stop()


