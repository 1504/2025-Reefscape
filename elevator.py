#being worked on by emma, advik & chatur

import rev
import wpilib
from wpilib import TimedRobot, Joystick
import commands2
from commands2 import Command
from commands2 import Subsystem

class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #placeholder number
        self.Motor = rev.CANSparkMax(0, rev.CANSparkMax.MotorType.kBrushless)

    def levelOne(self):
        #certain angle
        pass
    
    def levelTwo(self):
        #certain angle
        pass

    def levelThree(self):
        #certain angle
        pass

    def levelFour(self):
        #certain angle
        pass
    
    def stop(self):
        self.Motor.set(0.0)

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

    def initialize(self):
        #idk what this is
        #self.elevator_subsystem.intake()
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
            self.elevator_subsystem.levelOne()
        if self.joystick.getRawButton(2):#palceholder
            self.elevator_subsystem.levelTwo()
        if self.joystick.getRawButton(2):#palceholder
            self.elevator_subsystem.levelThree()
        if self.joystick.getRawButton(2):#palceholder
            self.elevator_subsystem.levelFour()
        else:
            self.elevator_subsystem.stop()