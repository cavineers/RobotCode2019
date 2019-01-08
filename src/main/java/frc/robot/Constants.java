package frc.robot;

import frc.lib.pathPursuit.Lookahead;

public class Constants {

	//path pursuit stuff
	public static final double kPathPursuitTolerance = 1; //in inches; get within 1 inche of the endpoint prior to moving on to the next segment
	public static final double kPathPursuitFinishTolerance = 5;
	public static final double kDefaultDt = 0.05; // in seconds; the default dt for pathfinding calculations
	public static final double kMaxAccelSpeedUp = 120; //in inches/sec^2; the max acceleration the robot can be commanded to experience when traveling a path
	public static final double kStopSteeringDistance = 5; //in inches; when the robot stops steering after a path

	//physical info about the chassis
	public static final double kWheelDiameter = 6; // in inches; the diameter of the drive wheels
	public static final double kWheelBase = 30; //in inches; the distance between the left and right drive wheels
	
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
    
    //Talon stuff
	public static final int kTimeoutMs = 10;
	public static final int kPIDLoopIdx = 0;
	
	//PID stuff
	public static final double kPVelocity = 4.5;//1.8; //4
	public static final double kIVelocity = 0.0;//0.0; //0
	public static final double kDVelocity = 25;//0.15; //10
	public static final double kFVelocity = 0.9;//1.5//0.6; //2  //75 in/sec at 0.5 power // 53400 units/100ms
 	public static final int kVelocityIZone = 0;
 	public static final double kAVelocity = 0.0; // TODO: Tune/Test
 	public static final double kDriveVoltageRampRate = 0.0;
 	
 	public static final Lookahead lookahead = new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed);
    
}