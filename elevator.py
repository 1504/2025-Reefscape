import wpilib
from wpilib import TimedRobot, Joystick

from wpimath.controller import PIDController,ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
import math

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase

import commands2
from commands2 import Subsystem, Command

import constants

class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #motor controllers
        self.elevatorMotor1: SparkMax = SparkMax(9, SparkMax.MotorType.kBrushless)#both are same orientation
        self.elevatorMotor2: SparkMax = SparkMax(10, SparkMax.MotorType.kBrushless)

        
        

        #config variables
        self.elevatorMotorConfig = SparkMaxConfig()

        self.elevatorMotorConfig.inverted(True)


        self.rev_resets = rev.SparkMax.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkMax.PersistMode.kPersistParameters


        #encoders
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()

        self.elevatorEncoder1.setPosition(0)
        self.elevatorEncoder2.setPosition(0)
        #pid controllers 

        self.pidController1 = ProfiledPIDController(0.04, 0.0, 0.001,TrapezoidProfile.Constraints(0.05, 0.01))
        self.pidController2 = ProfiledPIDController(0.04, 0.0, 0.001,TrapezoidProfile.Constraints(0.05, 0.01))


    def goToTarget(self, target: float):

        self.pidController1.setGoal(target)


    def printHeight(self):
        print(self.elevatorEncoder1.getPosition())
    
    
    def up(self):
         self.elevatorMotor1.set(0.05)
         self.elevatorMotor2.set(0.05)

    def downManual(self):

        self.elevatorMotor1.set(-0.05)
        self.elevatorMotor2.set(-0.05)
    def down(self):

        self.elevatorMotor1.set(-0.15)
        self.elevatorMotor2.set(-0.15)

    
    #try to calibrate so default pos is 0
    def defaultPos(self):
        #while self.elevatorEncoder2.getPosition() > constants.kDefaultPosRotation:
           # self.elevatorMotor1.set(-constants.kDefaultPosSpeed)
           # self.elevatorMotor2.set(constants.kDefaultPosSpeed)
        self.elevatorEncoder1.setPosition(0)
        self.elevatorEncoder2.setPosition(0)
        # self.elevatorMotor1.set(0.0)
        # self.elevatorMotor2.set(0.0)

    def stop(self):
        self.elevatorMotor1.set(0.0)
        self.elevatorMotor2.set(0.0)



class ElevatorDownManualCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem
        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.downManual() 

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class ElevatorDownCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem
        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.down() 

    def end(self, interrupted):
        self.elevator_subsystem.stop()


class ElevatorL2Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.goToTarget(self,7.6)

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class ElevatorL3Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.goToTarget(self,16.3)

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class ElevatorL4Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.goToTarget(self,31.5)

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class UpCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.up()

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class printHeightCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
     #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.printHeight()

    def end(self, interrupted):
        self.elevator_subsystem.stop()