import wpilib
from wpilib import TimedRobot, Joystick

import math

import rev
from rev import SparkMax

import commands2
from commands2 import Subsystem, Command

import constants

class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #placeholder number
        self.elevatorMotor1: SparkMax = SparkMax(9, SparkMax.MotorType.kBrushless)#both are same orientation
        self.elevatorMotor2: SparkMax = SparkMax(10, SparkMax.MotorType.kBrushless)
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.pidController1 = self.elevatorMotor1.getClosedLoopController()
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()
        self.pidController2 = self.elevatorMotor1.getClosedLoopController()
        
        # Initial gains
        self.pidController1.closedLoop.P(0.1)
        self.pidController1.closedLoop.I(0.01)
        self.pidController1.closedLoop.D(0.001)
        #self.pidController1.closedLoop.velocityFF(0)
        #self.pidController1.closedLoop.outputRange(0,0)

        # Initial gains
        self.pidController2.closedLoop.P(0.1)
        self.pidController2.closedLoop.I(0.01)
        self.pidController2.closedLoop.D(0.001)
        #self.pidController2.closedLoop.velocityFF(0)
        #self.pidController2.closedLoop.outputRange(0,0)

    #fix this later lol
        
    #def set_setpointl1(self, setpointl1):
        #self.setpointl1 = 60 #placeholder value for l1 setpoint
    
    def l1(self):
        current_position = self.elevatorEncoder1.get()
        control_effort = self.pidController1.calculate(current_position, 60)
        self.elevatorMotor1.set(control_effort)
        self.elevatorMotor2.set(control_effort)

    def up(self):
        self.elevatorMotor1.set(-0.25)
        self.elevatorMotor2.set(-0.25)
    def down(self):

        self.elevatorMotor1.set(0.12)
        self.elevatorMotor2.set(0.12)

    
    #try to calibrate so default pos is 0
    def defaultPos(self):
        #while self.elevatorEncoder2.getPosition() > constants.kDefaultPosRotation:
           # self.elevatorMotor1.set(-constants.kDefaultPosSpeed)
           # self.elevatorMotor2.set(constants.kDefaultPosSpeed)
        self.elevatorMotor1.set(0.0)
        self.elevatorMotor2.set(0.0)

    def stop(self):
        self.elevatorMotor1.set(0.0)
        self.elevatorMotor2.set(0.0)

class ElevatorUpCommand(Command):
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

class ElevatorL1Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.l1()

    def end(self, interrupted):
        self.elevator_subsystem.stop()