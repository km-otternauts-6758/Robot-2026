import sys
import wpilib
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController
from rev import SparkMax
from elasticlib import Notification, NotificationLevel, send_notification
from time import sleep as eep
import math

from Autonomous.commands.Modes import brady


# from phoenix6 import controls, configs, hardware, signals


from AutoSequencerV2.autoSequencer import AutoSequencer
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.drivetrainCommand import DrivetrainCommand
from humanInterface.driverInterface import DriverInterface

# from humanInterface.ledControl import LEDControl
from Subsystems.chlimechite import LimeLight
from Subsystems.thehappenings import Shooter, Intake, Shoulder
from Subsystems.pidshizz import PIDShizz

from wpilib import SmartDashboard

# from subsystems.presets import Presets
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate
from utils.singleton import destroyAllSingletonInstances
from utils.constants import kRPM
from webserver.webserver import Webserver


kLEDBuffer = 150



class MyRobot(wpilib.TimedRobot):
    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        # self.enableLiveWindowInTest(True)

        #########################################################
        # self.kraken = hardware.TalonFX(13)
        #self.seesTag = Notification(NotificationLevel.INFO, "Sees Tag", "I see a tag")
        self.timer = wpilib.Timer()
        self.smartdashboardbullcrap = PIDShizz()
        self.intake = Intake(14)
        self.shoulder = Shoulder(20)
        self.shooter = Shooter(13, 17, 15)


        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.

        self.webserver = Webserver()

        self.limelight = LimeLight()

        self.driveTrain = DrivetrainControl()
        self.autodrive = AutoDrive()

        self.stt = SegmentTimeTracker()

        self.dInt = DriverInterface()
        self.stick = self.dInt.ctrl
        self.stick2 = wpilib.XboxController(1)

        self.autoSequencer = AutoSequencer()

        self.autoHasRun = False 
        self.rpm = kRPM
        
        # test PID Shizz
        self.drivePid = PIDController(2.5, 0, 0.02, 0.01)
        self.drivePidX = PIDController(0.04, 0, 0.004, 0.01)
        self.drivePidY = PIDController(0.08, 0, 0.008, 0.01)
        # If this is in the final code, "remove" Scott's right to code (kill him)

    def robotPeriodic(self):

        #################### AUTO DISTANCE STUFF ################################
        """This code is stolen from a limelight guide Scott Bartz and Brady Decker saw back in 2024.  It works pretty well.
            I can't explain how it works but it works!
        """
        targetOffsetAngleVertical = -self.limelight.tx()
        limelightMountAngleDegrees = 20
        limelightLensHeightInches = 6.5
        goalHeightInches = 44

        self.angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngleVertical
        angleToGoalRadians = self.angleToGoalDegrees * (3.14159 / 180.0)

        self.distance = (goalHeightInches - limelightLensHeightInches) / math.tan(angleToGoalRadians)


        #################### ADJUSTMENTS FOR THE PID STUFF ################################

        """These are the actual equations for each """

        self.hoodPIDGoal = (-1.27 + (0.000564*(self.distance)) + (0.000229*(math.pow(self.distance, 2))))
        
        self.shooterRPMGoal = (-44.4 - (0.404*(self.distance)) + (0.00218*(math.pow(self.distance, 2))))

        SmartDashboard.putNumber("Hood PID", self.hoodPIDGoal)
        SmartDashboard.putNumber("Shooter PID", self.shooterRPMGoal)
        #########################################################
        self.smartdashboardbullcrap.updater()
        self.stt.start()

        self.dInt.update()
        self.stt.mark("Driver Interface")

        if self.stick.getRawButton(8):    
            self.driveTrain.update(True)
        else:
            self.driveTrain.update(False)
        
        self.stt.mark("Drivetrain")

        self.autodrive.updateTelemetry()
        self.driveTrain.poseEst._telemetry.setCurAutoDriveWaypoints(
            self.autodrive.getWaypoints()
        )
        self.driveTrain.poseEst._telemetry.setCurObstacles(
            self.autodrive.rfp.getObstacleStrengths()
        )
        self.stt.mark("Telemetry")

        logUpdate()
        self.stt.end()
        

    def autonomousInit(self):
        self.timer.start()
        self.driveTrain.resetGyro()
        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        # Use the autonomous routines starting pose to init the pose estimator
        self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())
        # self.driveTrain.poseEst.setKnownPose(Pose2d(1.0, 1.0, Rotation2d(0.0)))

        # Mark we at least started autonomous
        self.autoHasRun = True

        self.timer.restart()

    def autonomousPeriodic(self):
        self.autoSequencer.update()

    def autonomousExit(self):
        self.autoSequencer.end()
        # print("auto done.")

    ## Teleop-Specific init and update
    def teleopInit(self):
        self.shooter.setPosition(0)
        # clear existing telemetry trajectory
        self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)

        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if not self.autoHasRun:
            self.driveTrain.poseEst.setKnownPose(Pose2d(1.0, 1.0, Rotation2d(0.0)))
            print("not running.")
        self.timer.restart()
        self.rpm = 0

    def teleopPeriodic(self):
        
        SmartDashboard.putNumber("RPM", self.rpm)
        SmartDashboard.putNumber("Distance", self.distance)
        SmartDashboard.putNumber("Hood", self.shooter.getAimerPosition())
        SmartDashboard.putNumber("Shooter Speed", self.shooter.getShooterVelocity())
        # LIMELIGHT ALIGN (Left Side Values: TX: 27.58 TY: 5.74) (Right Side Values: TX: -10.6 TY: 8.37)
        print("Hood Position: ", self.shooter.getAimerPosition())
        if self.stick.getPOV() == 0:
            self.rpm += 1.66
            eep(.1)
        elif self.stick.getPOV() == 180:
            self.rpm -= 1.66
            eep(.1)
        print("RPM: ", self.rpm)
        print("TX ", self.limelight.tx())
        
        #print(self.stick.getPOV(90), self.stick.getPOV(180))

        if self.stick.getRawAxis(3):
            # self.shooter.setAimerSpeed(self.shooter.calculateAimer(self.shooter.getAimerPosition(), self.hoodPIDGoal))
            # self.shooter.calculateShooter(self.shooterRPMGoal)
            self.shooter.setShooterSpeed(self.shooter.calculateShooter(self.rpm))
        else:
            self.shooter.setShooterSpeed(0)

        if self.stick.getRawButton(2):
            self.shoulder.set(0.3)
            print("no")
        elif self.stick.getRawButton(3):
            self.shoulder.set(-0.3)
            print("no")
        else:
            self.shoulder.set(0)

        if self.stick.getRawButton(1):
            self.intake.set(0.75)
        elif self.stick.getRawButton(4):
            self.intake.set(-0.75)
        else:
            self.intake.set(0)

        # if self.stick.getRawButton(7):
        #     self.driveTrain.resetGyro()



        #print({"Aimer Current Position", self.shooter.getAimerPosition()})
        if self.stick.getRawButton(5):
            self.shooter.setAimerSpeed(
                -0.5
                # self.shooter.calculateAimer(self.shooter.getAimerPosition(),  -1.00244140625)
            )
        elif self.stick.getRawButton(6):
            self.shooter.setAimerSpeed(
                0.5
                # self.shooter.aimerPID.calculate(self.shooter.getAimerPosition(), 0.56298828125)
            )
        elif self.stick.getRawButton(7):
            self.shooter.setAimerSpeed(self.shooter.calculateAimer(self.shooter.getAimerPosition(), 0))
        else:
            self.shooter.setAimerSpeed(0)


        # if self.stick.getRawButton(8):
        #     self.driveTrain.followLimelight()
        # else:
        self.driveTrain.setManualCmd(self.dInt.getCmd())
            

        print(self.limelight.ty())

        

        # No trajectory in Teleop
        Trajectory().setCmd(None)

        #########################################################

    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()


    def disabledInit(self):
        self.autoSequencer.updateMode(True)

    ## Cleanup
    def endCompetition(self):
        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        destroyAllSingletonInstances()
        super().endCompetition()

