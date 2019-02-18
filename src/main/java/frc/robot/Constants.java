package frc.robot;

import frc.lib.Vector3D;

public class Constants {

    //path pursuit stuff
    public static final double kPathPursuitTolerance = 1; //in inches; get within 1 inch of the endpoint prior to moving on to the next segment
    public static final double kPathPursuitFinishTolerance = 5;
    public static final double kDefaultDt = 0.01; // in seconds; the default dt for pathfinding calculations
    public static final double kMaxAccelSpeedUp = 120; //in inches/sec^2; the max acceleration the robot can be commanded to experience when traveling a path
    public static final double kStopSteeringDistance = 3; //in inches; when the robot stops steering after a path

    //physical info about the chassis
    public static final double kWheelDiameter = 6; // in inches; the diameter of the drive wheels
    public static final double kWheelBase = 35; //in inches; the distance between the left and right drive wheels
    
    //pulses per inch of the wheel encoders
    public static final double kSensorUnitsPerInch = 854.3 / 12; //pulses per foot 854.3 math; 876.7 experimentally, 848.18- 4 times 64

    //Variable lookahead stuff
    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 9.0; // inches per second
    public static final double kMaxLookAhead = 24.0; // inches
    public static final double kMaxLookAheadSpeed = 120.0; // inches per second

    //turn to angle stuff
    public static final double kMaxTurnToAngleSpeed = 144; // in inches per second; the max speed of the robot used for turn to angle
    public static final double kAngleTolerance = 0.5; // in degrees; the tolerance of turn to angle
    
    //elevator stuff
    public static final double kElevatorPosTolerance = 0.1;
    public static final double kElevatorMaxAcceleration = 40; //800
    public static final double kElevatorMaxSpeed = 70; //3000
    public static final double kElevatorMaxHeight  = 42000; //TODO: set actual max height (rotations)
	public static final double kElevatorMinHeight = 0; //TODO: set actual min height (rotations)
    public static final double kElevatorRotationsPerInch = .178;

    //TODO: set these values
    public static final double kElevatorGroundLvl = 0; 
    public static final double kElevatorHatchPickupLvl = 0;
    public static final double kElevatorCargoPickupLvl = 0;
    public static final double kElevatorHomeHeight = -.25;
    public static final double kElevatorHomeHeightRotations = kElevatorHomeHeight*kElevatorRotationsPerInch;
    
    public static final double kElevatorLvl1Hatch = (4.5*kElevatorRotationsPerInch) + Math.abs(kElevatorHomeHeightRotations);
    public static final double kElevatorLvl1Cargo = (3*kElevatorRotationsPerInch) + Math.abs(kElevatorHomeHeightRotations);
    public static final double kElevatorLvl2Hatch = (32.5*kElevatorRotationsPerInch) + Math.abs(kElevatorHomeHeightRotations);
    public static final double kElevatorLvl2Cargo = (31*kElevatorRotationsPerInch) + Math.abs(kElevatorHomeHeightRotations);
    public static final double kElevatorLvl3Hatch = (60.5*kElevatorRotationsPerInch) + Math.abs(kElevatorHomeHeightRotations);
    public static final double kElevatorLvl3Cargo = (58.5*kElevatorRotationsPerInch) + Math.abs(kElevatorHomeHeightRotations);

    //On NEO
    public static final double kPVelocityElev = 0.0005;
    public static final double kIVelocityElev = 0;
    public static final double kDVelocityElev = 0.00001;
    public static final double kFVelocityElev = 0.000285714;
    
    public static final double kElevPIDPosPeriod = .025; //seconds
    public static final double kElevPercentTolerance = 5;
    
    //On RIO
    public static final double kPPosElev = 1000;
    public static final double kIPosElev = 0;
    public static final double kDPosElev = 800;
    public static final double kFPosElev = 0;

    public static final double kElevSensorLocation = 0;

    public static final double kMaxMoveGrabber = 0;
    public static final double kMinMoveGrabber = 0;

    //Constants for elevator homing
    public static final double kHomeTimeout = 180; //seconds
    public static final double kHomeMotorSpeed = -.1; //percent motor output
    public static final double kHomeCurrentThreshold = 5;
    public static final double kHomeEncoderVelTolerance = 1000;
    public static final int kHomeEncoderCurrentCycle = 250;

    
    //Talon stuff
    public static final int kTimeoutMs = 10;
    public static final int kPIDLoopIdx = 0;
    
    //Drive PID stuff
    // public static final double kPVelocity = 2;//1.8; //4
    // public static final double kIVelocity = 0.0;//0.0; //0
    // public static final double kDVelocity = 0.15;//0.15; //10
    // public static final double kFVelocity = 1.5;//1.5//0.6; //2  //75 in/sec at 0.5 power // 53400 units/100ms
    public static final double kPVelocity = 4.5;//1.8; //4
	public static final double kIVelocity = 0.0;//0.0; //0
	public static final double kDVelocity = 25;//0.15; //10
	public static final double kFVelocity = 0.9;//1.5//0.6
    public static final int kVelocityIZone = 0;
    public static final double kAVelocity = 0.0001; // TODO: Tune/Test
    public static final double kDriveVoltageRampRate = 0.0;
    
    //Position mapping stuff
    public static final int kMaxListSize = (int)(1 / Constants.kOdometryLoopTime) * 3; // max size of position lists is 

    public static final double kCameraToMapToleranceLvl1 = 1.5; // in inches, the max distance where the robot's position map can be completely 
                                                                // rebased by a camera update

    public static final double kCameraToMapToleranceLvl2 = 3.0; // in inches, the max distance where the robot's position map can be partially 
                                                                // rebased by a camera update with the lvl 2 scale factor
    public static final double kCameraToMapPercentLvl2 = 0.5; // the portion of the camera update rebased when the updates are within lvl 2 tolerance
                                                              // of current map estimates

    public static final double kCameraToMapToleranceLvl3 = 8.0; // in inches, the max distance where the robot's position map can be partially 
                                                                // rebased by a camera update with the lvl 3 scale factor
    public static final double kCameraToMapPercentLvl3 = 0.2; // the portion of the camera update rebased when the updates are within lvl 3 tolerance
                                                                // of current map estimates
    // Loop times
    public static final double kClockSyncLoopTime = 1; // check for clock sync once per second
    public static final double kOdometryLoopTime = 0.05; // update odometry 20 times per second
    public static final double kTargetingUpdatePeriod = 0.05;

    //Auto Targeting stuff
    public static final double kMaxTargetSpeed = 50; //in inches per second; max speed of the robot when it is targeting vision tape
    public static final double kMaxTargetAccel = 120;

    // public static final double kMinLookAheadTargeting = 3.0; // inches
    // public static final double kMinLookAheadSpeedTargeting = 12.0; // inches per second
    // public static final double kMaxLookAheadTargeting = 10.0; // inches
    // public static final double kMaxLookAheadSpeedTargeting = 120.0; // inches per second

    public static final double kMinLookAheadTargeting = 3.0; // inches
    public static final double kMinLookAheadSpeedTargeting = 3.0; // inches per second
    public static final double kMaxLookAheadTargeting = 12.0; // inches
    public static final double kMaxLookAheadSpeedTargeting = 50.0; // inches per second


    public static final double kStraightLineDistance = 2; // distance in inches of the straight line path at the end of a target
    public static final double kMinRadiusTargeting = 5; // radius in inches of the minimum allowed radius for a dubin's path

    // Robot Vector constants
    // TODO get actual measurements

    // a vector representing the difference in position from the robot's center of mass and the camera
    public static final Vector3D kCameraRelativeToRobotCenter = new Vector3D(0, 0, 0); 

    //a 3x3 rotation matrix that converts from the camera coordinate frame to the robot coordinate frame
    public static final double[][] kCameraToRobotMatrix = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}; //assumes no difference in rotation from the robot

    // Intake
    public static final double kCargoIntakeSpeed = 1;

    // Climber //TODO: TUNE
    public static final double kClimberRPI = 1; // Rotations per inch
    public static final double kClimberDeployPos = 10.0; // inches
    public static final double kClimberTolerance = 1;

    // Climber PID //TODO: TUNE ONCE WE HAVE A CLIMBER
    public static final double kPClimber = 2; 
    public static final double kIClimber = 0;
    public static final double kDClimber = 0;
    public static final double kFClimber = 0;
    public static final double kIZoneClimber = 0;

    public static final double kClimberMaxOutput = 1;
    public static final double kClimberMinOutput = -1;

    public static final double kClimbTime = 40.0; //earliest time to deploy climber (seconds)

    public static final double kClimbLegDeployed = 0;
    public static final double kClimbLegRetracted = 0;
    public static final double kClimbArmDeployed = 0;
    public static final double kClimbArmRetracted = 0;


    // Grabber
    public static final double kGrabberHomingSpeed = -0.1;
    public static final double kGrabberMaxHomingCurrent = 10; // once the grabber motor exceeds this when homing, it will be considered homed
    public static final double kGrabberHomePos = -0.05; //TODO set // the position of the hard stop that the elevator stalls against

    public static final double kGrabberPIDPosPeriod = .025;

    // grabber movement speeds
    public static final double kGrabberIntakeSpeed = 0.1;
    public static final double kGrabberEjectionSpeed = 1;
    public static final double kGrabberEjectionTime = 1;

    public static final double kGrabberTolerance = 0.1; //TODO set

    public static final double kGrabberRetractedPos = 0; //TODO set
    public static final double kGrabberExtendedPos = 0; //TODO set
    
    public static final double kGrabberStartPos = 0; //TODO: set

    public static final double kGrabberVelP = 0;
    public static final double kGrabberVelI = 0;
    public static final double kGrabberVelD = 0;
    public static final double kGrabberVelF = 0;

    public static final double kGrabberPosP = 0;
    public static final double kGrabberPosI = 0;
    public static final double kGrabberPosD = 0;
    public static final double kGrabberPosF = 0;

    public static final double kGrabberMaxSpeed = 20; //TODO
    public static final double kGrabberPercentTolerance = 5;

    public static final double kMinGrabberPos = 0; //TODO
    public static final double kMaxGrabberPos = 0; //TODO



    //Digital In/Output mapping

    //auto selection
    public static final int kAutoPos0 = 0;
    public static final int kAutoPos1 = 1;
    public static final int kAutoPos2 = 2;
    public static final int kAutoPos3 = 3;
    public static final int kAutoPos4 = 4;

    public static final int kDIO5 = 5;
    public static final int kDIO6 = 6;
    public static final int kDIO7 = 7;
    public static final int kGrabberHatchLimitSwitch = 8;
    public static final int kGrabberCargoLimitSwitch = 9;
    
}