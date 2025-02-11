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
        self.elevatorMotor: SparkMax = SparkMax(0, SparkMax.MotorType.kBrushless)
        self.elevatorEncoder = self.elevatorMotor.getEncoder()

    def l1(self):
        while self.elevatorEncoder.getPosition() < constants.kL1RotationDistance:
            self.elevatorMotor.set(constants.kL1RotationSpeed)
        self.elevatorMotor.set(0.0)
    
    def l2(self):
        while self.elevatorEncoder.getPosition() < constants.kL2RotationDistance:
            self.elevatorMotor.set(constants.kL2RotationSpeed)
        self.elevatorMotor.set(0.0)
    
    def l3(self):
        while self.elevatorEncoder.getPosition() < constants.kL3RotationDistance:
            self.elevatorMotor.set(constants.kL3RotationSpeed)
        self.elevatorMotor.set(0.0)

    def l4(self):
        while self.elevatorEncoder.getPosition() < constants.kL4RotationDistance:
            self.elevatorMotor.set(constants.kL4RotationSpeed)
        self.elevatorMotor.set(0.0)

    #try to calibrate so default pos is 0
    def defaultPos(self):
        while self.elevatorEncoder.getPosition() > constants.kDefaultPosRotation:
            self.elevatorMotor.set(constants.kDefaultPosSpeed)
        self.elevatorMotor.set(0.0)

    def stop(self):
        self.elevatorMotor.set(0.0)

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