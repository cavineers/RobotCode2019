/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.lib.MathHelper;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.HatchScoop;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CargoIntake;
import frc.robot.DankDash;

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
  public static Climber climber;
  public static HatchScoop hatchScoop;
  public static CargoIntake cargoIntake;
  public static Grabber grabber;

  public DankDash dankDash;

  public static AHRS gyro;
  public static OI oi;
  public static RobotPosEstimator estimator;

  public static CameraHelper reflectiveTapeCamera;

  public static CameraManager cameraManager;

  public static LEDHelper leds;

  // Camera clock sync checking thread
  Notifier clockSyncUpdater = new Notifier(this::checkForClockSync);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // initialize subsystems
    drivetrain = new DriveTrain();
    elevator = new Elevator();
    climber = new Climber();
    hatchScoop = new HatchScoop();
    cargoIntake = new CargoIntake();
    grabber = new Grabber();

    //initialize gyro
    gyro = new AHRS(SPI.Port.kMXP);

    //initialize operator interface / controls
    oi = new OI();

    //start estimating position of the robot
    estimator = new RobotPosEstimator(0, 0, 0, drivetrain.getRightPos(), drivetrain.getLeftPos());

    //initialize sensors
    reflectiveTapeCamera = new CameraHelper("reflectiveTape");
    gyro.zeroYaw();
    gyro.setAngleAdjustment(0);

    // start up the camera manager
    cameraManager = new CameraManager();

    // start up the led manager
    leds = new LEDHelper();

    //begin positional estimation
    estimator.start(); 

    //start ensuring that vision coprocessor(s) have properly synchronized clocks
    clockSyncUpdater.startPeriodic(Constants.kClockSyncLoopTime);

    // Init and export profile to network tables
    dankDash = new DankDash();
    dankDash.setProfileLocation("TestChassis");
    dankDash.setProfileName("Test Chassis");
    dankDash.export();

    }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    cameraManager.checkForCamUpdates();
    leds.update();
    heartbeat();
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // System.out.println(estimator.getPos().position);
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
        System.out.println("should sync clocks");
        reflectiveTapeCamera.startClockSync();
    }
  }

  /**
   * Returns true if the robot is in climber mode
   */
  public static boolean isClimbing() {
    return false; //TODO: implement
  }

  public void heartbeat() {
    if (Robot.getCurrentTime() - lastHeartbeatTime > 1) {
        NetworkTable netTable = NetworkTableInstance.getDefault().getTable("DankDash");
        lastHeartbeatTime = Robot.getCurrentTime();
        netTable.getEntry("Heartbeat").setDouble(heartbeatValue);
        heartbeatValue++;
    }
  }
}