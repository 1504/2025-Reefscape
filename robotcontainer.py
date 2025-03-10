#bye i cant
import time
from pathplannerlib.auto import AutoBuilder, NamedCommands
import wpilib 
import commands2
from commands2 import Commands, PrintCommand

import elevator

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """
    def __init__(self) -> None:

        self.elevator_subsystem = elevator.ElevatorSubsystem()
        
        self.register_commands()
        
        self.initialize_dashboard()
        
        #Pathfinding.setPathfinder(LocalADStar())

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def initialize_dashboard(self):
        #Choosing an Autonomous Program########################
        #Now, in autonomousInit and autonomousPeriodic, you can use the m_autoSelected variable to read which option was chosen, and change what happens during the autonomous period.
        self.auto_chooser = AutoBuilder.buildAutoChooser()
        self.auto_chooser.setDefaultOption('Wait', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(15)))
        #self.auto_chooser.addOption('Drive by velocity leave', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)))
        wpilib.SmartDashboard.putData('Auto choices', self.auto_chooser)

    def register_commands(self):
        NamedCommands.registerCommand('ElevatorL2Command', elevator.ElevatorL2Command(self.elevator_subsystem))

    def get_autonomous_command(self):
        # return DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)
        return self.auto_chooser.getSelected()