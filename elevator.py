import wpilib
from wpilib import TimedRobot, Joystick

import math

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase

import commands2
from commands2 import Subsystem, Command

import constants

class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #placeholder number
        self.elevatorMotor1: SparkMax = SparkMax(9, SparkMax.MotorType.kBrushless)#this one is inverted
        self.elevatorMotor2: SparkMax = SparkMax(10, SparkMax.MotorType.kBrushless)#bottom
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()

    def up(self):
        # emma's code commented out
        # while self.elevatorEncoder2.getPosition() < constants.kL1RotationDistance:
        #     # self.elevatorMotor1.set(-constants.kL1RotationSpeed)
        #     self.elevatorMotor2.set(constants.kL1RotationSpeed)
        # self.elevatorMotor1.set(0.0)
        # self.elevatorMotor2.set(0.0)
        self.elevatorMotor1.set(0.1)
        self.elevatorMotor2.set(-0.1)
        pass

    
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

class ElevatorCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem
        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.up()
        pass 

    def end(self, interrupted):
        self.elevator_subsystem.stop()