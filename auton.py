import wpilib
import wpilib.drive
import commands2
import time

class DriveStraight(commands2.CommandBase):
    def __init__(self, drive, distance, speed):
        super().__init__()
        self.drive = drive
        self.distance = distance
        self.speed = speed
        self.addRequirements([drive])
        self.start_time = 0

    def initialize(self):
        self.start_time = time.time()

    def execute(self):
        self.drive.arcadeDrive(-0.5*self.speed, 0)
    
    def end(self, interrupted):
         self.drive.arcadeDrive(0, 0)

    def isFinished(self):
        return time.time() - self.start_time >= self.distance

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.left_motor = wpilib.PWMVictorSPX(0)
        self.right_motor = wpilib.PWMVictorSPX(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.command_scheduler = commands2.CommandScheduler.getInstance()

    def autonomousInit(self):
       self.command_scheduler.schedule(DriveStraight(self.drive, 2, 0.5)) # Drive straight for 2 seconds at half speed

    def autonomousPeriodic(self):
        self.command_scheduler.run()

    #def teleopPeriodic(self):
    #    self.drive.arcadeDrive(self.joystick.getY(), self.joystick.getX())