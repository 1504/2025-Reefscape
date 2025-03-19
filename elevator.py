import wpilib
from wpilib import TimedRobot, Joystick

from wpimath.controller import PIDController,ProfiledPIDController
from constants import ElevatorConstants
import wpimath.trajectory

import math
from wpimath.units import inchesToMeters

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase

import commands2
from commands2 import Subsystem, Command

import constants

class ElevatorSubsystem(commands2.TrapezoidProfileSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                0.05,
                0.01
            ),
            initial_position=1.3,
            period=0.02
        )
        self.setName("PenguinGoUpElevator")
        self.counter = ElevatorConstants.k_counter_offset
        self.is_moving = False  # may want to keep track of if we are in motion
        self.tolerance = 0.03  # meters - then we will be "at goal"
        self.goal = 1.3
        self.at_goal = True
        self.feedforward = wpimath.controller.ElevatorFeedforward(
            kS=ElevatorConstants.k_kS_volts,
            kG=ElevatorConstants.k_kG_volts,
            kV=ElevatorConstants.k_kV_volt_second_per_radian,
            kA=ElevatorConstants.k_kA_volt_second_squared_per_meter,
            dt=0.02)

        #motor controllers
        self.motor: SparkMax = SparkMax(9, SparkMax.MotorType.kBrushless)#both are same orientation
        self.follower: SparkMax = SparkMax(10, SparkMax.MotorType.kBrushless)

        self.sparks = [self.motor,self.follower]

        self.rev_resets = rev.SparkMax.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkMax.PersistMode.kPersistParameters
        # this should be its own function later - we will call it whenever we change brake mode
        self.motor.configure(ElevatorConstants.k_config, self.rev_resets, self.rev_persists)
        self.follower.configure(ElevatorConstants.k_follower_config, self.rev_resets, self.rev_persists)

        # configure our PID controller
        self.controller = self.motor.getClosedLoopController()
        # does this still work?
        # self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(self.goal)

        self.enable()   
        

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.velocity/2)  # the 2 corrects for the 2x carriage speed

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid
    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            ElevatorConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            ElevatorConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        else:
            ElevatorConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
            ElevatorConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.motor.configure(ElevatorConstants.k_config, self.rev_resets, self.rev_persists)
        self.follower.configure(ElevatorConstants.k_follower_config, self.rev_resets, self.rev_persists)

    def get_height(self):
        return self.encoder.getPosition()

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal function
        goal = goal if goal < ElevatorConstants.k_max_height else ElevatorConstants.k_max_height
        goal = goal if goal > ElevatorConstants.k_min_height else ElevatorConstants.k_min_height
        self.goal = goal
        # print(f'setting goal to {self.goal}')
        self.setGoal(self.goal)
        self.at_goal = False

    def move_meters(self, delta_meters: float, silent=False) -> None:  # way to bump up and down for testing
        current_position = self.get_height()
        goal = current_position + delta_meters
        self.set_goal(goal)  # check and set
        if not silent:
            message = f'setting {self.getName()} from {current_position:.2f} to {self.goal:.2f}'
            print(message)

    def get_at_goal(self):
        return self.at_goal

    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.position = self.encoder.getPosition()
            self.at_goal = math.fabs(self.position - self.goal) < self.tolerance  # maybe we want to call this an error
            self.error = self.position - self.goal




class ElevatorSmartCommand(commands2.Command):
    def __init__(self,elevator: ElevatorSubsystem,height=inchesToMeters(8),offset=0,mode='specified', wait_to_finish=False, indent=0) -> None:
        super().__init__()
        self.setName('Move Elevator')  # change this to something appropriate for this command
        self.indent = indent
        self.elevator = elevator
        self.height = height
        self.offset = offset  # attempt to have an offset
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.elevator)  # commandsv2 version of requirements

        # sick of IDE complaining
        self.start_time = None
        self.goal = None

    def initialize(self) -> None:
            """Called just before this Command runs the first time."""
            self.start_time = round(self.container.get_enabled_time(), 2)
            if  self.mode == 'specified':  # send to a specific height
                self.goal = self.height
                self.elevator.set_goal(self.goal)
            elif self.mode == 'incremental':  # call from GUI to increment up and down
                self.goal = self.height  # height is a delta in this case
                self.elevator.move_meters(delta_meters=self.goal)
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.elevator.get_at_goal() # TODO - put in a timeout, and probably a minimum time to allow to start moving
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            #SmartDashboard.putString(f"alert",
            #                         f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

class ElevatorL2Command(Command):
    def __init__(self, elevator_subsystem):
        super().__init__()

        self.elevator_subsystem = elevator_subsystem

        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.elevator_subsystem.goToTarget(self,7.6)

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
        self.elevator_subsystem.goToTarget(self,16.3)

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
        self.elevator_subsystem.goToTarget(self,31.5)

    def end(self, interrupted):
        self.elevator_subsystem.stop()

class UpCommand(Command):
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