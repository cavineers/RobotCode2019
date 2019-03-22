/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.lib.MathHelper;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ControlType;

import frc.robot.subsystems.HatchScoop;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Grabber.HatchGrabberState;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.CargoIntake;
import frc.robot.DankDash;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoPathHelper.PATH_TARGET;
import frc.robot.AutoPathHelper.START_POS;
import frc.robot.commands.HomeAll;
import frc.robot.commands.auto.DisableAutoOverride;
import frc.robot.commands.auto.OverrideAutoSelection;
import frc.robot.commands.auto.TwoHatchCargoLeft;
import frc.robot.commands.auto.TwoHatchCargoRight;
import frc.robot.commands.auto.TwoHatchRocketLeft;
import frc.robot.commands.auto.TwoHatchRocketRight;
import frc.robot.commands.grabber.ToggleHatchGrabber;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // Subsystems
    public static DriveTrain drivetrain;
    public static Elevator elevator;
    public static HatchScoop hatchScoop;
    public static CargoIntake cargoIntake;
    public static Grabber grabber;

    public DankDash dankDash;

    public static double lastUpdateTime;
    public static double lastLEDUpdateTime;

    public static NetworkTable netTable;
    public static double heartbeatValue;

    public static AHRS gyro;
    public static OI oi;
    public static RobotPosEstimator estimator;

    public static CameraHelper reflectiveTapeCamera;

    public static LEDHelper leds;

    public static boolean isAutoOverridden = false;

    public static START_POS overriddenStartPos = START_POS.INVALID;
    public static PATH_TARGET overriddenPathTarget = PATH_TARGET.FRONT_CARGOBAY;

    // Digital Inputs for auto selection
    DigitalInput rightStart, leftStart, middleStart;
    AnalogInput favorCargoBayFront, favorCargoBaySide, favorRocket;

    double posError;
    double posWant;
    double posGot;
    double velError;
    double velWant;
    double velGot;

    String posData;
    String velData;

    // Camera clock sync checking thread
    Notifier clockSyncUpdater = new Notifier(this::checkForClockSync);

    public static SendableChooser<START_POS> posChooser;
    public static SendableChooser<PATH_TARGET> targetChooser;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // initialize subsystems
        drivetrain = new DriveTrain();
        elevator = new Elevator();
        hatchScoop = new HatchScoop();
        cargoIntake = new CargoIntake();
        grabber = new Grabber();

        // initialize gyro
        gyro = new AHRS(SPI.Port.kMXP);

        // start estimating position of the robot
        estimator = new RobotPosEstimator(0, 0, 0, drivetrain.getRightPos(), drivetrain.getLeftPos());

        // initialize operator interface / controls
        oi = new OI();

        // initialize sensors
        reflectiveTapeCamera = new CameraHelper("reflectiveTape");
        gyro.zeroYaw();
        gyro.reset();

        // start up the led manager
        leds = new LEDHelper();

        // begin positional estimation
        estimator.start();

        // start ensuring that vision coprocessor(s) have properly synchronized clocks
        // clockSyncUpdater.startPeriodic(Constants.kClockSyncLoopTime); //TODO: uncomment if we use vision

        // Init and export profile to network tables
        dankDash = new DankDash();
        dankDash.setProfileLocation("TestChassis");
        dankDash.setProfileName("Test Chassis");
        dankDash.export();
        dankDash.addListener();

        LiveWindow.disableAllTelemetry();


        rightStart = new DigitalInput(0);
        middleStart = new DigitalInput(1);
        leftStart = new DigitalInput(2);
        

        favorCargoBayFront = new AnalogInput(0);
        favorCargoBaySide = new AnalogInput(1);
        favorRocket = new AnalogInput(2);

        posChooser = new SendableChooser<START_POS>();
		posChooser.setDefaultOption("Middle", START_POS.MIDDLE);
		posChooser.addOption("Right", START_POS.RIGHT);
		posChooser.addOption("Left", START_POS.LEFT);
		posChooser.addOption("INVALID", START_POS.INVALID);
        SmartDashboard.putData(posChooser);
        
        targetChooser = new SendableChooser<PATH_TARGET>();
		targetChooser.setDefaultOption("Front CargoBay", PATH_TARGET.FRONT_CARGOBAY);
		targetChooser.addOption("Side CargoBay", PATH_TARGET.SIDE_CARGOBAY);
		targetChooser.addOption("Rocket", PATH_TARGET.ROCKET);
        SmartDashboard.putData(targetChooser);
        
        SmartDashboard.putData(new DisableAutoOverride());
		SmartDashboard.putData(new OverrideAutoSelection("SMARTDASH"));


        // Elevator

        // Velocity loop
        // SmartDashboard.putNumber("f-val", Constants.kFVelocityElevUp);
        // SmartDashboard.putNumber("p-val", Constants.kPVelocityElev);
        // SmartDashboard.putNumber("i-val", Constants.kIVelocityElev);
        // SmartDashboard.putNumber("d-val", Constants.kDVelocityElev);
        // SmartDashboard.putNumber("current-vel",
        // elevator.getElevatorMotor().getEncoder().getVelocity());

        // Position loop
        // SmartDashboard.putNumber("f-val", Constants.kFPosElev);
        // SmartDashboard.putNumber("p-val", Constants.kPPosElev);
        // SmartDashboard.putNumber("i-val", Constants.kIPosElev);
        // SmartDashboard.putNumber("d-val", Constants.kDPosElev);
        // SmartDashboard.putNumber("current-pos",
        // elevator.getElevatorMotor().getEncoder().getPosition());
        // SmartDashboard.putNumber("position output: ", elevator.getPIDPosOutput());

        // //Grabber
        // SmartDashboard.putNumber("f-val", Constants.kGrabberVelF);
        // SmartDashboard.putNumber("p-val", Constants.kGrabberVelP);
        // SmartDashboard.putNumber("i-val", Constants.kGrabberVelI);
        // SmartDashboard.putNumber("d-val", Constants.kGrabberVelD);
        // SmartDashboard.putNumber("grabber-vel", grabber.getArmMotor().getEncoder().getVelocity());

        // SmartDashboard.putNumber("f-val", Constants.kGrabberPosF);
        // SmartDashboard.putNumber("p-val", Constants.kGrabberPosP);
        // SmartDashboard.putNumber("i-val", Constants.kGrabberPosI);
        // SmartDashboard.putNumber("d-val", Constants.kGrabberPosD);
        // SmartDashboard.putNumber("grabber-pos", grabber.getArmMotor().getEncoder().getPosition());

        //Tuning ball intake PID
        // SmartDashboard.putNumber("f-val", Constants.kGrabberBallVelF);
        // SmartDashboard.putNumber("p-val", Constants.kGrabberBallVelP);
        // SmartDashboard.putNumber("i-val", Constants.kGrabberBallVelI);
        // SmartDashboard.putNumber("d-val", Constants.kGrabberBallVelD);
        // SmartDashboard.putNumber("wheel vel", grabber.getBallMotor().getSelectedSensorVelocity());


    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // this.updateLEDs(); //TODO: uncomment if we use leds
        this.updateDankDash();
        // For tuning elevator:

        // For figuring out homing:
        // SmartDashboard.putBoolean("elevatorSwitch", elevator.getLimitSwitch());

        // Velocity:
        // elevator.getElevatorMotor().getPIDController().setFF(SmartDashboard.getNumber("f-val",
        // 0));
        // elevator.getElevatorMotor().getPIDController().setP(SmartDashboard.getNumber("p-val",
        // 0));
        // elevator.getElevatorMotor().getPIDController().setI(SmartDashboard.getNumber("i-val",
        // 0));
        // elevator.getElevatorMotor().getPIDController().setD(SmartDashboard.getNumber("d-val",
        // 0));
        // SmartDashboard.putNumber("current-vel",
        // elevator.getElevatorMotor().getEncoder().getVelocity());

        // //Position:
        // elevator.getPIDPos().setF(SmartDashboard.getNumber("f-val", 0));
        // elevator.getPIDPos().setP(SmartDashboard.getNumber("p-val", 0));
        // elevator.getPIDPos().setI(SmartDashboard.getNumber("i-val", 0));
        // elevator.getPIDPos().setD(SmartDashboard.getNumber("d-val", 0));

        // SmartDashboard.putNumber("current-pos",
        // elevator.getElevatorMotor().getEncoder().getPosition());

        // Grabber:

        // //position PID
        // grabber.getArmMotor().getPIDController().setFF(SmartDashboard.getNumber("f-val", 0));
        // grabber.getArmMotor().getPIDController().setP(SmartDashboard.getNumber("p-val", 0));
        // grabber.getArmMotor().getPIDController().setI(SmartDashboard.getNumber("i-val", 0));
        // grabber.getArmMotor().getPIDController().setD(SmartDashboard.getNumber("d-val", 0));
        // SmartDashboard.putNumber("grabber-vel", grabber.getArmMotor().getEncoder().getVelocity());

        // grabber.pidPos.setF(SmartDashboard.getNumber("f-val", 0));
        // grabber.pidPos.setP(SmartDashboard.getNumber("p-val", 0));
        // grabber.pidPos.setI(SmartDashboard.getNumber("i-val", 0));
        // grabber.pidPos.setD(SmartDashboard.getNumber("d-val", 0));
        SmartDashboard.putNumber("grabber-pos", grabber.getPosition());
        SmartDashboard.putNumber("elevator-pos", elevator.getPosition());
        SmartDashboard.putNumber("ball vel", grabber.getBallMotor().getSelectedSensorVelocity());

        // SmartDashboard.putNumber("grabber-current",
        // grabber.getArmMotor().getOutputCurrent());
        SmartDashboard.putString("grabber-level", String.valueOf(grabber.getState()));
        SmartDashboard.putString("elevator-level", String.valueOf(Robot.elevator.getLevel()));
        // SmartDashboard.putString("can-move-elev",

        // grabber.getBallVelPID().setF(SmartDashboard.getNumber("f-val", 0));
        // grabber.getBallVelPID().setP(SmartDashboard.getNumber("p-val", 0));
        // grabber.getBallVelPID().setI(SmartDashboard.getNumber("i-val", 0));
        // grabber.getBallVelPID().setD(SmartDashboard.getNumber("d-val", 0));
        // SmartDashboard.putNumber("wheel vel", grabber.getBallMotor().getSelectedSensorVelocity());

        SmartDashboard.putBoolean("Hatch Switch", Robot.grabber.hasHatch());
        SmartDashboard.putBoolean("Cargo Switch", Robot.grabber.hasCargo());
        SmartDashboard.putBoolean("Grabber Homing Switch", Robot.grabber.isAtHome());
        SmartDashboard.putBoolean("Elevator Homing", Robot.elevator.getLimitSwitch());

        SmartDashboard.putBoolean("Elevator Homing", Robot.elevator.getLimitSwitch());

        SmartDashboard.putString("Hatch Grabber State", String.valueOf(Robot.grabber.getHatchGrabberState()));
        SmartDashboard.putString("Cargo Intake Position", String.valueOf(Robot.cargoIntake.getPosition()));
        SmartDashboard.putString("Can move grabber", String.valueOf(Robot.elevator.canMoveGrabber()));

        if(this.isEnabled()  && Robot.grabber.hasHatch() && Robot.grabber.getHatchGrabberState()==HatchGrabberState.INTAKING && (Math.abs(Robot.getCurrentTime()-Robot.grabber.getLastToggleTime()) < Constants.kGrabberAutoToggleTolerance)){
            new ToggleHatchGrabber();
        }
    
    }

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    @Override
    public void disabledInit() {
        Scheduler.getInstance().removeAll();
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        SmartDashboard.putString("Robot Position Start: ", String.valueOf(getAutoStartPos()));
        SmartDashboard.putString("Robot Auto Target: ", String.valueOf(getAutoPathTarget()));
        dankDash.sendDash("RobotPositionStart", String.valueOf(getAutoStartPos()));
        dankDash.sendDash("RobotAutoTarget", String.valueOf(getAutoPathTarget()));
    }

    public START_POS getAutoStartPos() {
        if (isAutoOverridden) {
            return overriddenStartPos;
        } else {
            if (!leftStart.get() && rightStart.get() && middleStart.get()) {
                return START_POS.LEFT;
            } else if (leftStart.get() && !rightStart.get() && middleStart.get()) {
                return START_POS.RIGHT;
            } else if (leftStart.get() && rightStart.get() && !middleStart.get()) {
                return START_POS.MIDDLE;
            }  else {
                return START_POS.INVALID;
            }
        }
    }

    public PATH_TARGET getAutoPathTarget() {
        if (isAutoOverridden) {
            return overriddenPathTarget;
        } else {
            boolean cargoBayFront = favorCargoBayFront.getVoltage() > 4;
            boolean cargoBaySide = favorCargoBaySide.getVoltage() > 4;
            boolean rocket = favorRocket.getVoltage() > 4;
            if (!cargoBayFront && cargoBaySide && !rocket) {
                return PATH_TARGET.SIDE_CARGOBAY;
            } else if (cargoBayFront && !cargoBaySide && !rocket) {
                return PATH_TARGET.FRONT_CARGOBAY;
            } else if (!cargoBayFront && !cargoBaySide && rocket) {
                return PATH_TARGET.ROCKET;
            } else {
                //default ot front cargobay if there is an error
                return PATH_TARGET.FRONT_CARGOBAY;
            }
        }
    }

    public Command getAutoCommand() {
        START_POS startPos = this.getAutoStartPos();
        PATH_TARGET target = this.getAutoPathTarget();
        if (startPos == START_POS.RIGHT) {
            if (target == PATH_TARGET.FRONT_CARGOBAY) {
                return new TwoHatchCargoRight();
            } else if (target == PATH_TARGET.SIDE_CARGOBAY) {
                return new TimedCommand(0); //implement!
            } else if (target == PATH_TARGET.ROCKET) {
                // return new TwoHatchRocketRight();
                return new TimedCommand(0);
            }
        } else if (startPos == START_POS.LEFT) {
            if (target == PATH_TARGET.FRONT_CARGOBAY) {
                return new TwoHatchCargoLeft();
            } else if (target == PATH_TARGET.SIDE_CARGOBAY) {
                return new TimedCommand(0); //implement!
            } else if (target == PATH_TARGET.ROCKET) {
                // return new TwoHatchRocketLeft();
                return new TimedCommand(0);
            }
        } else if (startPos == START_POS.MIDDLE) {
            return new TimedCommand(0); //implement!
        }

        return new TimedCommand(0); //the robot was in an invalid state! do nothing
    }
    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString code to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        new HomeAll().start();
        getAutoCommand().start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        oi.updatePeriodicCommands();
        Scheduler.getInstance().run();
        
    }

    @Override
    public void teleopInit() {
        
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        oi.updatePeriodicCommands();
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * Get the current heading of the robot in radians
     * 
     * @return the current robot heading in radians from -pi to pi
     */
    public static double getAngleRad() {
        return MathHelper.angleToNegPiToPi(Math.toRadians(Robot.gyro.getYaw()));
    }

    /**
     * Gets the current time in seconds
     * 
     * @return the current time in seconds
     */
    public static double getCurrentTime() {
        return Timer.getFPGATimestamp();
    }

    /**
     * Returns true if the robot is currently in the end game period
     */
    public static boolean isEndGame() {
        return DriverStation.getInstance().isOperatorControl() && DriverStation.getInstance().getMatchTime() < 30;
    }

    /**
     * Resyncs clocks if needed
     */
    protected void checkForClockSync() {
        if (reflectiveTapeCamera.shouldSyncClocks()) {
            reflectiveTapeCamera.startClockSync();
        }
    }

    public void updateLEDs() {
        if (Robot.getCurrentTime() - lastLEDUpdateTime > 0.25) { // update the LEDs 4 times per second
            lastLEDUpdateTime = Robot.getCurrentTime();
            leds.update();
        }
    }

    /**
     * Sends important values like the match time and the heartbeat value to the
     * dashbaord
     */
    public void updateDankDash() {
        if (Robot.getCurrentTime() - lastUpdateTime > 1) { // update the dashboard display once per second
            lastUpdateTime = Robot.getCurrentTime();
            dankDash.sendDash("Heartbeat", Double.toString(heartbeatValue));
            heartbeatValue++;
            dankDash.sendDash("MatchTime", Double.toString(DriverStation.getInstance().getMatchTime()));
        }
    }

}