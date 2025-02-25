import wpilib
from wpilib import TimedRobot, Joystick

from wpimath.controller import PIDController
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

        #self.elevatorEncoder1.setPosition(0)
        #self.elevatorEncoder2.setPosition(0)

        self.elevatorMotor1.setInverted(True)
        self.elevatorMotor2.setInverted(True)

        #config variables
        self.elevatorMotor1Config = SparkMaxConfig()
        self.elevatorMotor2Config = SparkMaxConfig()

        #encoders
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()

        #pid controllers 
        #self.pidController1 = self.elevatorMotor1.getClosedLoopController()
        #self.pidController2 = self.elevatorMotor2.getClosedLoopController()
        #self.elevatorMotor1Config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        #self.elevatorMotor2Config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.pidController1 = PIDController(0.02,0.01,0.004)
        self.pidController2 = PIDController(0.02,0.01,0.004)

        # Initial gains
        #self.elevatorMotor1Config.closedLoop.P(.1)
        #self.elevatorMotor1Config.closedLoop.I(0)
        #self.elevatorMotor1Config.closedLoop.D(0)
        #self.elevatorMotor1Config.closedLoop.velocityFF(0.1)
        #self.elevatorMotor1Config.closedLoop.outputRange(-1,1)#what does this do diego

        # Initial gains
        #self.elevatorMotor2Config.closedLoop.P(.1)
        #self.elevatorMotor2Config.closedLoop.I(0)
        #self.elevatorMotor2Config.closedLoop.D(0)
        #self.elevatorMotor2Config.closedLoop.velocityFF(0.1)
        #self.elevatorMotor2Config.closedLoop.outputRange(-1,1)

    #fix this later lol
        
    #def set_setpointl1(self):#, setpointl1):
        #self.setpointl1 = setpointl1
        #self.pidController1.setReference(self.setpointl1, rev.ControlType.kPosition)
        #self.pidController2.setReference(self.setpointl1, rev.ControlType.kPosition)
    
    def l1(self):
        #current_position = self.elevatorEncoder1.getPosition()
        #control_effort = self.pidController1.setReference(current_position, -10)
        #self.elevatorMotor1.set(control_effort)
        #self.elevatorMotor2.set(control_effort)
        #self.setpointl1 = -10

        #self.elevatorMotor1Config.closedLoop.P(self.kP)
        #self.elevatorMotor2Config.closedLoop.P(self.kP)
        
        #self.pidController1.setReference(10, rev.SparkMax.ControlType.kPosition)
        self.elevatorMotor1.set(-1*self.pidController1.calculate(10, self.elevatorEncoder1.getPosition()))
        self.elevatorMotor2.set(-1*self.pidController1.calculate(10, self.elevatorEncoder1.getPosition()))
        #self.pidController2.setReference(10, rev.SparkMax.ControlType.kPosition)
        #self.pidController2.setReference(self.setpointl1, rev.SparkMax.ControlType.kPosition)
        #print("Position")
        #print(self.elevatorEncoder1.getPosition())
        #print(self.elevatorEncoder2.getPosition())
        #print(self.elevatorMotor1.getBusVoltage())
        #print("Calculated")
        #print(self.pidController1.calculate(10, self.elevatorEncoder1.getPosition()))


    def printHeight(self):
        print(self.elevatorEncoder1.getPosition())



    def up(self):
        self.elevatorMotor1.set(0.1)
        self.elevatorMotor2.set(0.1)
    def down(self):

        self.elevatorMotor1.set(-0.1)
        self.elevatorMotor2.set(-0.1)

    
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