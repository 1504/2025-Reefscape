# import rev
# import wpilib
# from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
# import commands2
# from commands2 import Subsystem, Command
# from rev import SparkMax, SparkMaxConfig, SparkBase
# import math
# import constants
# import drivesubsystem

# class AutonSubsystem(Subsystem):
#     def __init__(self):
#         super().__init__()
        
#         self.swerve = drivesubsystem()
#         self.timer = wpilib.Timer()

#         #conveyorDetector.whileActiveContinuous(new RunMotor());
#         #self.motors = wpilib.MotorControllerGroup(self.leftMotor, self.rightMotor)

#     def forward(self):
#         self.swerve.drive(-0.2, 0, 0, False, True)

#     def stop(self):
#         self.swerve.drive(0, 0, 0, False, True)

# class forwardCommand(Command):
#     def __init__(self, auton_subsystem):
#         super().__init__()

#         self.auton_subsystem = auton_subsystem

#     def initialize(self):
#         pass

#     def execute(self):
#         self.auton_subsystem.forward()

#     def end(self, interrupted):
#         self.auton_subsystem.stop()
