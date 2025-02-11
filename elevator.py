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
        self.elevatorMotor2: SparkMax = SparkMax(10, SparkMax.MotorType.kBrushless)
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()

    def l1(self):
        while self.elevatorEncoder2.getPosition() < constants.kL1RotationDistance:
            self.elevatorMotor1.set(-constants.kL1RotationSpeed)
            self.elevatorMotor2.set(constants.kL1RotationSpeed)
        self.elevatorMotor1.set(0.0)
        self.elevatorMotor2.set(0.0)
    
    #try to calibrate so default pos is 0
    def defaultPos(self):
        while self.elevatorEncoder2.getPosition() > constants.kDefaultPosRotation:
            self.elevatorMotor1.set(-constants.kDefaultPosSpeed)
            self.elevatorMotor2.set(constants.kDefaultPosSpeed)
        self.elevatorMotor1.set(0.0)
        self.elevatorMotor2.set(0.0)

    def stop(self):
        self.elevatorMotor1.set(0.0)
        self.elevatorMotor2.set(0.0)

    #necessity of these 2?
    def periodic(self):
        # This method will be called once per scheduler run
        pass
    def simulationPeriodic(self):
        # This method will be called once per scheduler run during simulation
        pass

class ElevatorCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem
        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        pass 

    def end(self, interrupted):
        self.elevator_subsystem.stop()

    #necessity of these 2
    def periodic(self):
        # This method will be called once per scheduler run
        pass
    def simulationPeriodic(self):
        # This method will be called once per scheduler run during simulation
        pass

class Robot(TimedRobot):
    def robotInit(self):
        self.joystick = Joystick(0)#placeholder
        self.elevator_subsystem = ElevatorSubsystem()
        self.elevator_command = ElevatorCommand(self.elevator_subsystem)

    def teleopPeriodic(self):
        if self.joystick.getRawButton(1):#palceholder
            self.elevator_subsystem.l1()
        else:
            self.elevator_subsystem.stop()