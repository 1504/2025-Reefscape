#being worked on by emma, advik & chatur

'''Import'''
import wpilib
import math

import rev

import commands2
from commands2 import Subsystem

import constants

class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #placeholder number
        self.elevatorMotor = rev.CANSparkMax(0, rev.CANSparkMax.MotorType.kBrushless)
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
        self.elevator_subsystem.intake()

    def execute(self):
        pass 

    def end(self, interrupted):
        self.intake_subsystem.stop()

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
        self.intake_subsystem = IntakeSubsystem()
        self.intake_command = IntakeCommand(self.intake_subsystem)

    def teleopPeriodic(self):
        if self.joystick.getRawButton(1):#palceholder
            self.intake_subsystem.intake()
        else:
            self.intake_subsystem.stop()