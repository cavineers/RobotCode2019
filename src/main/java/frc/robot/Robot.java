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
import edu.wpi.first.wpilibj.command.Scheduler;
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
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.CargoIntake;
import frc.robot.DankDash;
import frc.robot.commands.elevator.HomeElev;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        // initialize operator interface / controls
        oi = new OI();

        // start estimating position of the robot
        estimator = new RobotPosEstimator(0, 0, 0, drivetrain.getRightPos(), drivetrain.getLeftPos());

        // initialize sensors
        reflectiveTapeCamera = new CameraHelper("reflectiveTape");
        gyro.zeroYaw();
        gyro.setAngleAdjustment(0);

        // start up the led manager
        leds = new LEDHelper();

        // begin positional estimation
        estimator.start();

        // start ensuring that vision coprocessor(s) have properly synchronized clocks
        clockSyncUpdater.startPeriodic(Constants.kClockSyncLoopTime);

        // Init and export profile to network tables
        dankDash = new DankDash();
        dankDash.setProfileLocation("TestChassis");
        dankDash.setProfileName("Test Chassis");
        dankDash.export();

        SmartDashboard.putData(new HomeElev());

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
        // SmartDashboard.putNumber("f-val", Constants.kGrabberPosF);
        // SmartDashboard.putNumber("p-val", Constants.kGrabberPosP);
        // SmartDashboard.putNumber("i-val", Constants.kGrabberPosI);
        // SmartDashboard.putNumber("d-val", Constants.kGrabberPosD);
        // SmartDashboard.putNumber("current-vel",
        // elevator.getElevatorMotor().getEncoder().getVelocity());
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

        // Dank Dashboard stuff //TODO: uncomment if we use dank dashboard

        // posWant = Robot.elevator.getPIDPos().getSetpoint();
        // posGot = Robot.elevator.getElevatorMotor().getEncoder().getPosition();
        // posError = Math.abs(posWant-posGot);

        // velWant = Robot.elevator.getPIDPosOutput();
        // velGot = Robot.elevator.getElevatorMotor().getEncoder().getVelocity();
        // velError = Math.abs(velWant-velGot);

        // posData = Double.toString(posError) + "," + Double.toString(posWant) + "," +
        // Double.toString(posGot);
        // velData = Double.toString(velError) + "," + Double.toString(velWant) + "," +
        // Double.toString(velGot);
        // dankDash.sendDash("/DankDash/Pos", posData);
        // dankDash.sendDash("vel", velData);

        // SmartDashboard.putNumber("elevator vel",
        // elevator.getElevatorMotor().getEncoder().getVelocity());

        // For tuning elevator:

        // For figuring out homing:
        SmartDashboard.putBoolean("elevatorSwitch", elevator.getLimitSwitch());

        // Velocity:
        // elevator.getElevatorMotor().getPIDController().setFF(SmartDashboard.getNumber("f-val",
        // 0));
        // elevator.getElevatorMotor().getPIDController().setP(SmartDashboard.getNumber("p-val",
        // 0));
        // elevator.getElevatorMotor().getPIDController().setI(SmartDashboard.getNumber("i-val",
        // 0));
        // elevator.getElevatorMotor().getPIDController().setD(SmartDashboard.getNumber("d-val",
        // 0));
        SmartDashboard.putNumber("current-vel", elevator.getElevatorMotor().getEncoder().getVelocity());

        // //Position:
        // elevator.getPIDPos().setF(SmartDashboard.getNumber("f-val", 0));
        // elevator.getPIDPos().setP(SmartDashboard.getNumber("p-val", 0));
        // elevator.getPIDPos().setI(SmartDashboard.getNumber("i-val", 0));
        // elevator.getPIDPos().setD(SmartDashboard.getNumber("d-val", 0));
        SmartDashboard.putNumber("current-pos", elevator.getElevatorMotor().getEncoder().getPosition());

    //Grabber:
    
    //limit switches:
    SmartDashboard.putBoolean("hasHatch", grabber.hasHatch());
    SmartDashboard.putBoolean("hasCargo", grabber.hasCargo());

    // //position PID
    // grabber.getArmMotor().getPIDController().setFF(SmartDashboard.getNumber("f-val", 0));
    // grabber.getArmMotor().getPIDController().setP(SmartDashboard.getNumber("p-val", 0));
    // grabber.getArmMotor().getPIDController().setI(SmartDashboard.getNumber("i-val", 0));
    // grabber.getArmMotor().getPIDController().setD(SmartDashboard.getNumber("d-val", 0));
    SmartDashboard.putNumber("grabber-pos", grabber.getPosition());
    SmartDashboard.putNumber("grabber-current", grabber.getArmMotor().getOutputCurrent());
    
    SmartDashboard.putString("grabber-level", String.valueOf(grabber.getState()));
    SmartDashboard.putString("elevator-level", String.valueOf(elevator.getLevel()));
    SmartDashboard.putString("can-move-elev", String.valueOf(elevator.canMoveGrabber()));

}

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
      
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
      grabber.setEncoderPosition(Constants.kGrabberStartPos);
      grabber.getArmMotor().getPIDController().setReference(Constants.kGrabberStartPos, ControlType.kPosition);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    oi.updateDPadCommands();
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
   * @return the current robot heading in radians from -pi to pi
   */
  public static double getAngleRad() {
    return MathHelper.angleToNegPiToPi(Math.toRadians(Robot.gyro.getAngle()));
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

  /**
   * Returns true if the robot is in climber mode
   */
  public static boolean isClimbing() {
    return false; //TODO: implement
  }

  public void updateLEDs() {
    if (Robot.getCurrentTime() - lastLEDUpdateTime > 0.25) { // update the LEDs 4 times per second
        lastUpdateTime = Robot.getCurrentTime();
        leds.update();
    } 
  }

  /**
   * Sends important values like the match time and the heartbeat value to the dashbaord
   */
  public void updateDankDash() {
    if (Robot.getCurrentTime() - lastUpdateTime > 1) { // update the dashboard display once per second
        lastUpdateTime = Robot.getCurrentTime();
        dankDash.sendDash("Heartbeat", Double.toString(heartbeatValue));
        heartbeatValue++;
    }
  }

}