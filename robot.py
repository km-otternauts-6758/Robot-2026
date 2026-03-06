import sys
import wpilib
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController
#from wpilib import Ultrasonic
from rev import SparkMax
from elasticlib import Notification, NotificationLevel, send_notification
from time import sleep as eep
import math

import choreo
import choreo.util
from choreo import trajectory
from choreo.trajectory import EventMarker
from commands2 import InstantCommand



# from phoenix6 import controls, configs, hardware, signals


from AutoSequencerV2.autoSequencer import AutoSequencer
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.drivetrainCommand import DrivetrainCommand
from humanInterface.driverInterface import DriverInterface

# from humanInterface.ledControl import LEDControl
from Subsystems.chlimechite import LimeLight
from Subsystems.thehappenings import Shooter, BaseKraken, BaseNeo
from Subsystems.pidshizz import PIDShizz

from wpilib import SmartDashboard, SendableChooser

# from subsystems.presets import Presets
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate
from utils.singleton import destroyAllSingletonInstances
from utils.constants import kRPM
from webserver.webserver import Webserver




kLEDBuffer = 150
kP = 0.2
kI = 0.05
kD= 0.1


class MyRobot(wpilib.TimedRobot):
    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        # self.enableLiveWindowInTest(True)



        # LEDS
        self.led = wpilib.AddressableLED(1)
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(kLEDBuffer)]
        self.rainbowFirstPixelHue = 0
        self.led.setLength(kLEDBuffer)
        self.led.setData(self.ledData)
        self.led.start()






        #########################################################
        # self.kraken = hardware.TalonFX(13)
        #self.seesTag = Notification(NotificationLevel.INFO, "Sees Tag", "I see a tag")
        self.timer = wpilib.Timer()
        self.smartdashboardbullcrap = PIDShizz()
        self.intake = BaseNeo(14)
        self.shoulder = BaseNeo(20)
        self.climber = BaseNeo(18)
        self.shooter = Shooter(13, 17, 15)
        self.shooter.setPosition(0)
        self.indexer = BaseNeo(16)

        
        # self.choreoTrajectory = 
        self.autoChooser = SendableChooser()

        self.autoChooser.setDefaultOption("Goated", 1)
        self.autoChooser.addOption("BackUp", 2)
        self.autoChooser.addOption("OtherSide", 3)
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
 


    


        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.



        self.limelight = LimeLight()

        self.driveTrain = DrivetrainControl()
        self.autodrive = AutoDrive()

        self.stt = SegmentTimeTracker()

        self.dInt = DriverInterface()
        self.stick = self.dInt.stick
        self.stick2 = wpilib.XboxController(2)

        self.autoSequencer = AutoSequencer()

        self.autoHasRun = False 
        self.rpm = kRPM
        

        self.choreo1 = choreo.load_swerve_trajectory("Goated")
        self.choreo2 = choreo.load_swerve_trajectory("BackUp")
        self.choreo3 = choreo.load_swerve_trajectory("OtherSide")

        self.limelightPID = PIDController(kP, kI, kD)
        self.limelightPID.setTolerance(5.0)

        




    def robotPeriodic(self):

        #################### AUTO DISTANCE STUFF ################################
        """This code is stolen from a limelight guide Scott Bartz and Brady Decker saw back in 2024.  It works pretty well.
            I can't explain how it works but it works!
        """
        targetOffsetAngleVertical = self.limelight.ty()
        limelightMountAngleDegrees = 20
        limelightLensHeightInches = 10.5
        goalHeightInches = 44.5

        self.angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngleVertical
        angleToGoalRadians = self.angleToGoalDegrees * (3.14159 / 180.0)

        self.distance = (goalHeightInches - limelightLensHeightInches) / math.tan(angleToGoalRadians)


        #################### ADJUSTMENTS FOR THE PID STUFF ################################

        """These are the actual equations for each 
            Using a line of best fit graph you get these equations
            Basically just program manual commands for whatever you want to have an equation for
            then just test different numbers at different spots until you get what you want. 
        """

        self.hoodPIDGoal = (-0.882 + (-0.0196*(self.distance)) + (0.000446*(math.pow(self.distance, 2))))
        
        self.shooterRPMGoal = (-49.6 + (-0.123*(self.distance)) + (0.000916*(math.pow(self.distance, 2))))

        SmartDashboard.putNumber("Hood PID", self.hoodPIDGoal)
        SmartDashboard.putNumber("Shooter PID", self.shooterRPMGoal)
        SmartDashboard.putBoolean("Has Auton Run", self.autoHasRun)
        #########################################################
        self.smartdashboardbullcrap.updater()
        self.stt.start()

        self.dInt.update()
        self.stt.mark("Driver Interface")

           
        self.driveTrain.update()
        
        

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

        # if not self.stick.getRawButton(6) or self.stick.getRawButton(4):
        #     self.shooter.setAimerSpeed(self.shooter.calculateAimer(self.hoodPIDGoal))
        # elif self.hoodPIDGoal > 0.0 and self.shooter.getAimerPosition() > 0.0:
        #     self.shooter.setAimerSpeed(0)

        self.auton = self.autoChooser.getSelected()

        if self.auton == 1:
            if self.choreo1:
                self.autoPose = self.choreo1.get_final_pose(False)
                self.auto = self.choreo1
        elif self.auton == 2:
            if self.choreo2:
                self.autoPose = self.choreo2.get_final_pose(False)
                self.auto = self.choreo2
        elif self.auton == 3:
            if self.choreo3:
                self.autoPose = self.choreo3.get_final_pose(False)
                self.auto = self.choreo3
        

    def autonomousInit(self):
        self.driveTrain.resetGyro()
        # Start up the autonomous sequencer

        self.autoSequencer.initialize()
        

        # Use the autonomous routines starting pose to init the pose estimator
        # self.autoSequencer.getStartingPose()
        
        

        # Mark we at least started autonomous
        self.autoHasRun = True
        self.runThrough = 1
        self.timer.restart()
        SmartDashboard.putNumber("Auton Timer", self.timer.get())
        

    def autonomousPeriodic(self):
        if self.auto:
            sample = self.auto.sample_at(self.timer.get(), self.is_red_alliance())
            
            if sample:
                vx = sample.vx
                vy = sample.vy
                vOmega = sample.omega
        newCommand = DrivetrainCommand(vx, vy, vOmega)
        self.autoSequencer.update()
        self.driveTrain.setManualCmd(newCommand)
        # while self.timer.get() < .75:
        #     self.shoulder.set(0.3)

        # while self.timer.get() < 8.3 and self.timer.get() > 5.37:qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq
        #     self.intake.set(.75)

        if self.timer.get() > 5:
            self.shooter.setShooterSpeed(self.shooterRPMGoal)
            self.intake.set(1)
            self.indexer.set(-0.5)

        # if self.timer.get() <= self.auto.get_total_time():
        #     self.autodrive.setRequest(self.auto.sample_at(self.timer.get(), False))
        # else:
        #     self.autodrive.setRequest(None)


    def autonomousExit(self):
        if self.autoPose:
            self.driveTrain.poseEst.setKnownPose(self.autoPose)
        self.autoSequencer.end()
        # print("auto done.")

    ## Teleop-Specific init and update
    def teleopInit(self):
        # self.shooter.setPosition(0)
        # clear existing telemetry trajectory
        self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)
        self.autodrive.setRequest(None)
        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if not self.autoHasRun:
            self.driveTrain.poseEst.setKnownPose(Pose2d(1.0, 1.0, Rotation2d(0.0)))
            print("not running.")
        
        self.timer.restart()
        self.rpm = 0

    def teleopPeriodic(self):

        """Wes stuff"""
        a = self.stick2.getRawButton(1)
        b = self.stick2.getRawButton(2)
        x = self.stick2.getRawButton(3)
        y = self.stick2.getRawButton(4)
        shoot = self.stick2.getRawAxis(3)


        driverVX = -self.stick.getY()
        driverVY = -self.stick.getX()
        driverVOmega = self.limelightPID.calculate(self.limelight.tx(), 0)

        limelightAlign = DrivetrainCommand(driverVX, driverVY, driverVOmega)
        
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

        if shoot:
            # self.shooter.setAimerSpeed(self.shooter.calculateAimer(self.hoodPIDGoal))
            self.shooter.setShooterSpeed(self.shooter.calculateShooter(self.shooterRPMGoal))
            self.indexer.set(-0.5)
            self.color(0,0,255)
        # elif self.stick.getRawButton(7):
        #     self.shooter.setAimerSpeed(self.shooter.calculateAimer(self.shooter.getAimerPosition(), 0))
        else:
            self.shooter.setAimerSpeed(0)
            self.indexer.set(0)
            self.shooter.setShooterSpeed(0)

        if self.stick2.getRawButton(5):
            self.shoulder.set(0.5)
            print("no")
        elif self.stick2.getRawButton(6):
            self.shoulder.set(-0.5)
            print("no")
        else:
            self.shoulder.set(0)

        if a:
            self.indexer.set(-0.5)
            self.intake.set(1)
        elif b:
            self.indexer.set(0.5)
            self.intake.set(-1)
        else:
            self.intake.set(0)
            


        if x:
            self.climber.set(1)
        elif y:
            self.climber.set(-1)
        else:
            self.climber.set(0)

        if self.stick.getRawButton(9):
            self.driveTrain.resetGyro()



        #print({"Aimer Current Position", self.shooter.getAimerPosition()})
        

        """Enable when doing tests"""
        if self.stick.getRawButton(6):
            self.shooter.setAimerSpeed(1)
        elif self.stick.getRawButton(4):
            self.shooter.setAimerSpeed(-1)
        else:
            self.shooter.setAimerSpeed(0.0)
        


        if self.stick.getRawButton(8):
            self.driveTrain.setManualCmd(limelightAlign)
        else:
            self.driveTrain.setManualCmd(self.dInt.getCmd())

        

            

        # print(self.limelight.ty())

        

        # No trajectory in Teleop
        Trajectory().setCmd(None)

        #########################################################

    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()


    def disabledInit(self):
        self.autoSequencer.updateMode(True)


    def rainbow(self):
        for i in range(kLEDBuffer):
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180
            self.ledData[i].setHSV(int(hue), 255, 128)
        self.rainbowFirstPixelHue += 3
        self.rainbowFirstPixelHue %= 180

    def color(self, R: int, G: int, B: int):
        for i in range(kLEDBuffer):
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180
            self.ledData[i].setRGB(R, G, B)
        self.rainbowFirstPixelHue += 3
        self.rainbowFirstPixelHue %= 180

    def endLights(self):
        for i in range(kLEDBuffer):
            hue = self.timer.get() * 1.3
            self.ledData[i].setRGB(int(hue), int(hue), int(hue))

    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


    ## Cleanup
    def endCompetition(self):
        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        destroyAllSingletonInstances()
        super().endCompetition()

    


