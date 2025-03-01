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

        self.elevatorMotor1.setInverted(True)
        self.elevatorMotor2.setInverted(True)

        #config variables
        self.elevatorMotor1Config = SparkMaxConfig()
        self.elevatorMotor2Config = SparkMaxConfig()

        #encoders
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()

        self.elevatorEncoder1.setPosition(0)
        self.elevatorEncoder2.setPosition(0)

        #pid controllers 
        self.pidController1 = PIDController(0.04,0.0, 0.001)#.02 , 004 / .04, 0, 0
        self.pidController2 = PIDController(0.04,0.0, 0.001)

    
    
    #elevator levels, main stuff
     
    def down(self):
    #just down, faster than down adj, acts like l1, no pid
        self.elevatorMotor1.set(-0.15)
        self.elevatorMotor2.set(-0.15)
    
    def l2(self):
        self.elevatorMotor1.set((-1*self.pidController1.calculate(7.6, self.elevatorEncoder1.getPosition()))*0.5)
        self.elevatorMotor2.set((-1*self.pidController1.calculate(7.6, self.elevatorEncoder1.getPosition()))*0.5)
        print(self.elevatorEncoder1.getPosition())
    
    def l3(self):
        self.elevatorMotor1.set((-1*self.pidController1.calculate(16.3, self.elevatorEncoder1.getPosition()))*0.5)
        self.elevatorMotor2.set((-1*self.pidController1.calculate(16.3, self.elevatorEncoder1.getPosition()))*0.5)

    def l4(self):
        self.elevatorMotor1.set((-1*self.pidController1.calculate(31.5, self.elevatorEncoder1.getPosition()))*0.5)
        self.elevatorMotor2.set((-1*self.pidController1.calculate(31.5, self.elevatorEncoder1.getPosition()))*0.5)

 
    #elevator adjustments - for adjusting, temporary solution for the levels, hopefullly we can score by just holding the l# buttons.
    def upAdjustment(self):
         self.elevatorMotor1.set(0.05)
         self.elevatorMotor2.set(0.05)

    def downAdjustment(self):
        self.elevatorMotor1.set(-0.05)
        self.elevatorMotor2.set(-0.05)
   

    #try to calibrate so default pos is 0 ---- done.
    def defaultPos(self):
        self.elevatorEncoder1.setPosition(0)
        self.elevatorEncoder2.setPosition(0)

    def stop(self):
        self.elevatorMotor1.set(0.0)
        self.elevatorMotor2.set(0.0)

    def printHeight(self):
        print(self.elevatorEncoder1.getPosition())


#commands
class ElevatorDownAdjustmentCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()
        self.elevator_subsystem = elevator_subsystem
    def initialize(self):
        pass
    def execute(self):
        self.elevator_subsystem.downAdjustment() 

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class ElevatorDownCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()
        self.elevator_subsystem = elevator_subsystem
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
    def initialize(self):
        pass
    def execute(self):
        self.elevator_subsystem.l2()

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class ElevatorL3Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()
        self.elevator_subsystem = elevator_subsystem
    def initialize(self):
        pass
    def execute(self):
        self.elevator_subsystem.l3()

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class ElevatorL4Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()
        self.elevator_subsystem = elevator_subsystem        
    def initialize(self):
        pass
    def execute(self):
        self.elevator_subsystem.l4()

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class ElevatorUpAdjustmentCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()
        self.elevator_subsystem = elevator_subsystem
    #stopped here
    def initialize(self):
        pass
    def execute(self):
        self.elevator_subsystem.upAdjustment()

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class printHeightCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()
        self.elevator_subsystem = elevator_subsystem
    def initialize(self):
        pass
    def execute(self):
        self.elevator_subsystem.printHeight()

    def end(self, interrupted):
        self.elevator_subsystem.stop()