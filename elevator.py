import wpilib
from wpilib import TimedRobot, Joystick, Timer

from wpimath.trajectory import TrapezoidProfile

import wpimath.controller
from wpimath.controller import SimpleMotorFeedforwardMeters
import wpimath

from wpimath.controller import PIDController, ProfiledPIDController
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


        elevatorMotorConfig = SparkMaxConfig().inverted(True)

        #self.elevatorMotor1.configure(config=elevatorMotorConfig,resetMode=SparkMax.ResetMode.kNoResetSafeParameters,persistMode=SparkMax.PersistMode.kPersistParameters)
        #self.elevatorMotor2.configure(config=elevatorMotorConfig,resetMode=SparkMax.ResetMode.kNoResetSafeParameters,persistMode=SparkMax.PersistMode.kPersistParameters)
        #encoders
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()

        #pid controllers 

        # self.constraints = wpimath.trajectory.TrapezoidProfile.Constraints(1.75, 0.75)
        # self.controller = wpimath.controller.ProfiledPIDController(1.3, 0, 0.7, self.constraints, self.kDt)
        # self.elevatorEncoder1.setDistancePerPulse(1 / 360 * 2 * math.pi * 1.5)
        # self.lastSpeed = 0
        # self.lastTime = Timer.getFPGATimestamp()
        # self.feedforward = SimpleMotorFeedforwardMeters(kS= wpimath.volts, kV= wpimath.units.volt_seconds_per_meter,  kA= wpimath.units.volt_seconds_squared_per_meter, dt= wpimath.units.seconds)

        # Max velocity is 5 meters per second
        # Max acceleration is 5 meters per second
        self.pidController1 = ProfiledPIDController(0.018, 0.0, 0.00,TrapezoidProfile.Constraints(0.05, 0.01))
        self.pidController2 = ProfiledPIDController(0.018, 0.0, 0.00,TrapezoidProfile.Constraints(0.05, 0.01))


    
    def l2(self):
        #current_position = self.elevatorEncoder1.getPosition()
        # pidVal = self.pidController1.calculate(self.elevatorEncoder1.getDistance(), 7.7)
        # acceleration = (self.pidController1.getSetpoint().velocity - self.lastSpeed) / (Timer.getFPGATimestamp() - self.lastTime)
        # self.elevatorMotor1.setVoltage(pidVal + self.feedforward.calculate(self.pidController1.getSetpoint().velocity, acceleration))
        # self.elevatorMotor2.setVoltage(pidVal + self.feedforward.calculate(self.pidController1.getSetpoint().velocity, acceleration))

        # self.pidController1.setGoal(7.7)
        #self.elevatorMotor1.set(self.pidController1.calculate(self.elevatorEncoder1.getDistance(), 7.7))

        calculatedEffort = self.pidController1.calculate(goal=7.7,measurement=self.elevatorEncoder1.getPosition())
        self.elevatorMotor1.set(-calculatedEffort)
        self.elevatorMotor2.set(-calculatedEffort)

        # self.lastSpeed = self.pidController1.getSetpoint().velocity
        # self.lastTime = Timer.getFPGATimestamp()
    
    def l3(self):
        
        finalGoal = 16
        currentPosition = self.elevatorEncoder1.getPosition()

        ratio = currentPosition/finalGoal
        targetToAimFor = 0
        if ratio < 0.10:
            targetToAimFor = finalGoal * 0.25
        elif ratio < 0.75:
            targetToAimFor = finalGoal * 0.9
        elif ratio < 1.1:
            targetToAimFor = finalGoal


        self.elevatorMotor1.set(-1*self.pidController1.calculate(goal=targetToAimFor, measurement=self.elevatorEncoder1.getPosition()))
        self.elevatorMotor2.set(-1*self.pidController2.calculate(goal=targetToAimFor, measurement=self.elevatorEncoder1.getPosition()))
        print(self.elevatorEncoder1.getPosition())


    def l4(self):
        if self.elevatorEncoder1.getPosition() < 15:
            self.elevatorMotor1.set(-1*self.pidController1.calculate(goal=20, measurement=self.elevatorEncoder1.getPosition())*0.3)
            self.elevatorMotor2.set(-1*self.pidController1.calculate(goal=20, measurement=self.elevatorEncoder1.getPosition())*0.3)
        else:
            self.elevatorMotor1.set(-1*self.pidController1.calculate(goal=28, measurement=self.elevatorEncoder1.getPosition())*0.35)
            self.elevatorMotor2.set(-1*self.pidController1.calculate(goal=28, measurement=self.elevatorEncoder1.getPosition())*0.35)
        print(self.elevatorEncoder1.getPosition())
        #self.elevatorMotor1.set(-1*self.pidController1.calculate(31.5, self.elevatorEncoder1.getPosition())*0.3)
        #self.elevatorMotor2.set(-1*self.pidController1.calculate(31.5, self.elevatorEncoder1.getPosition())*0.3)

    def getspeed(self):
        print(self.elevatorEncoder1.getVelocity())
    



    def printHeight(self):
        print(self.elevatorEncoder1.getPosition())



    def up(self):
        self.elevatorMotor1.set(0.1)
        self.elevatorMotor2.set(0.1)

    def down(self):
        # instead of directly controlling the speed downwards, we are going to use the PID function as well
        # to avoid hard slam downwards
        self.elevatorMotor1.set(-1*self.pidController1.calculate(goal=1.3, measurement=self.elevatorEncoder1.getPosition()))
        self.elevatorMotor2.set(-1*self.pidController1.calculate(goal=1.3, measurement=self.elevatorEncoder1.getPosition()))

    
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

class ElevatorL2Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
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

        
    #stopped here
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

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):

        self.elevator_subsystem.l4()

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class getspeedCommand(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.getspeed()

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