import wpilib
import rev
import math
from math import pi
from rev import SparkMax, AbsoluteEncoder, SparkMaxConfig, EncoderConfig, SparkBase
from wpimath.controller import (
    PIDController,
    ElevatorFeedforward,
    ArmFeedforward,
    ProfiledPIDController,
)
from collections.abc import Callable

from wpiutil import Sendable

from phoenix6 import configs, hardware, signals
from phoenix6.controls import Follower

from wpilib import DigitalInput


# shooter VALUES
"""  PID is tuned by starting with each value at 0, then slowly increasing P, until it goes past your goal and then goes back forth a lot,
    then, start tuning D until it stops doing that.  THen increase I until it gets there at the speed you would like.
 """

shooterkP = .055
shooterkI = 0.012
shooterkD = 0.0


aimerkP = .6
aimerkI = 0
aimerkD = 0.0




class Shooter:
    def __init__(self, Flywheel1: int, Flywheel2: int, Aimer: int) -> None:

        """Flywheel: int and Aimer: int are how you tell it your can ids from another class.
        You call it the way, but you just give it 2 numbers, or in most cases 1.
        In the main code, it's called with Shooter(1, 2)."""
               
        # self.neo = Neo

        # if self.neo == False:
        self.krakenConfig = configs.TalonFXConfiguration()
        self.krakenConfig.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        self.shooter1 = hardware.TalonFX(Flywheel1)
        self.shooter1.configurator.apply(self.krakenConfig)
        self.shooter2 = hardware.TalonFX(Flywheel2)
        self.shooter2.set_control(Follower(self.shooter1.device_id, signals.MotorAlignmentValue.OPPOSED))
        
        # elif self.neo == True:
        #     self.shooter = SparkMax(Flywheel, SparkMax.MotorType.kBrushless)
        #     self.shooterEncoder = self.shooter.getEncoder()

        self.aimer = SparkMax(Aimer, SparkMax.MotorType.kBrushless)
        self.aimer.setInverted(False)

        self.aimerEncoder = self.aimer.getAlternateEncoder()

        self.shooterPID = PIDController(
            shooterkP, 
            shooterkI, 
            shooterkD
            )

        # self.shooterPid = PIDController(shooterkP, shooterkI, shooterkD)

        self.aimerPID = PIDController(
            aimerkP,
            aimerkI,
            aimerkD,
        )

        

    def setShooterSpeed(self, speed: float) -> None:
        self.shooter1.set(speed)
    def setAimerSpeed(self, speed: float) -> None:
        self.aimer.set(speed)
    

    def getAimerPosition(self) -> float:
        return self.aimerEncoder.getPosition()
    
    def setPosition(self, num: float):
        self.aimerEncoder.setPosition(num)
    
    def getShooterVelocity(self) -> float:
        
        return self.shooter1.get_velocity().value
        

    def calculateAimer(self, goal: float) -> float:
        aimerNumber = self.aimerPID.calculate(self.getAimerPosition(), goal)
        return aimerNumber
    def calculateShooter(self, goal: float) -> float:
        shooterNumber = self.shooterPID.calculate(self.getShooterVelocity(), goal)
        return shooterNumber

    def setSmartDashboard(self, setName: Callable[[Sendable, str, str], None]) -> None:
        setName(self.aimerPID, "shooterSubsystem", "PID")


class BaseKraken: 
    def __init__(self, Schlooper: int) -> None:
        self.kraken = hardware.TalonFX(Schlooper)
    def set(self, speed: float) -> None:
        self.kraken.set(speed)
class BaseNeo: 
    def __init__(self, Arm: int) -> None:
        self.neo = SparkMax(Arm, SparkMax.MotorType.kBrushless)
         
    def set(self, speed: float) -> None:
        self.neo.set(speed)

    # def getPosition(self) -> float:
    #     return self.shoulderEncoder.getOutput()