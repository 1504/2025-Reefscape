import rev
import wpilib
from wpilib import TimedRobot, Joystick
import commands2
from commands2 import Command
from commands2 import Subsystem

class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #0 & 1 are placeholder numbers
        self.leftMotor = rev.CANSparkMax(0, rev.CANSparkMax.MotorType.kBrushless)
        self.rightMotor = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.motors = wpilib.MotorControllerGroup(self.leftMotor, self.rightMotor)

    def intake(self):
        #placeholder values
        self.leftMotor.set(-1.0)
        self.rightMotor.set(1.0)
    
    #outtake?
    
    def stop(self):
        self.leftMotor.set(0.0)
        self.rightMotor.set(0.0)

    #necessity of these 2?
    def periodic(self):
        # This method will be called once per scheduler run
        pass
    def simulationPeriodic(self):
        # This method will be called once per scheduler run during simulation
        pass

class IntakeCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()

        self.intake_subsystem = intake_subsystem

    def initialize(self):
        self.intake_subsystem.intake()

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