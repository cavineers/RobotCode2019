package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.TankDriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
public class DriveTrain extends Subsystem {
	public WPI_TalonSRX leftMotor1  = new WPI_TalonSRX(RobotMap.leftDriveMotor1);
	public WPI_TalonSRX leftMotor2  = new WPI_TalonSRX(RobotMap.leftDriveMotor2);
	
	public WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightDriveMotor1);
	public WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightDriveMotor2);
	private DoubleSolenoid sol;
	
	private DifferentialDrive drive = new DifferentialDrive(leftMotor1, rightMotor1);
	
	private boolean mIsBrakeMode;

	public enum DriveGear {
		HIGH_GEAR,
		LOW_GEAR
	}

	public DriveTrain() {
		super();
		drive.setRightSideInverted(false);
		this.configTalons();
		leftMotor2.follow(leftMotor1);
		rightMotor2.follow(rightMotor1);
		sol = new DoubleSolenoid(RobotMap.PCM, 0, 1);
		this.setBrakeMode(false);
		this.setDriveGear(DriveGear.LOW_GEAR); //shift into low gear
	}

	/**
	 * Initialize normal driving as a default command
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TankDriveWithJoystick());
	}
	
	/**
	 * Set the current state of the gear shifting solenoid
	 * 
	 * @param gear: the desired gear to shift to
	 */
    public void setDriveGear(DriveGear gear){
    	if (gear == DriveGear.LOW_GEAR) {
    		sol.set(DoubleSolenoid.Value.kReverse);
    	} else if (gear == DriveGear.HIGH_GEAR) {
    		sol.set(DoubleSolenoid.Value.kForward);
    	}
    }
    
    /**
     * Get the current drive gear of the robot
     * 
     * @return if the DriveTrain is in high or low gear
     */
    public DriveGear getDriveGear() {
		if (sol.get() == DoubleSolenoid.Value.kReverse) {
			return DriveGear.LOW_GEAR;
		} else {
			return DriveGear.HIGH_GEAR;
		}
    }
    
	/**
	 * Tank style driving for the DriveTrain.
	 * 
	 * @param forward: forward velocity in range [-1,1]
	 * @param rotate: rotational velocity in range [-1,1]
	 */
	public void drive(double forward, double rotate) {
		drive.curvatureDrive(forward, rotate, true);
	}
	
	/**
	 * Drive the robot based on the current joystick state
	 * 
	 * @param joy: current state of the robot
	 */
	public void drive(Joystick joy) {
		this.drive(this.addDeadZone(-joy.getRawAxis(1)), this.addDeadZone(joy.getRawAxis(4)));
	}
	
	/**
	 * get the master talon of the left drive train
	 * @return the left master drive talon
	 */
	public WPI_TalonSRX getLeftTalon() {
		return this.leftMotor1;
	}

	/**
	 * get the master talon of the right drive train
	 * @return the right master drive talon
	 */
	public WPI_TalonSRX getRightTalon() {
		return this.rightMotor1;
	}
	
	/**
	 * Add dead zones to the value given by the joystick so that a slight bump does not
	 * move the robot.  Also square inputs for better velocity control
	 * 
	 * @param: input the raw value given by a joystick axis
	 */
	public double addDeadZone(double input) {
		if (Math.abs(input) <= .05)
			input = 0;
		else if (input < 0)
			input = -Math.pow(input, 2);
		else
			input = Math.pow(input, 2);
		return input;
	}
	
	/**
	 * configure the talons for use in the DriveTrain
	 */
	public void configTalons() {
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		
		rightMotor1.setInverted(false);
		rightMotor2.setInverted(false);
		
		rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutMs);
		rightMotor1.setSensorPhase(false); // keep encoder and motor in phase 
		
		// set the peak, nominal outputs
		rightMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightMotor1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightMotor1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// set closed loop gains in slot0
		rightMotor1.config_kP(Constants.kPIDLoopIdx, Constants.kPVelocity, Constants.kTimeoutMs);
		rightMotor1.config_kI(Constants.kPIDLoopIdx, Constants.kIVelocity, Constants.kTimeoutMs);
		rightMotor1.config_kD(Constants.kPIDLoopIdx, Constants.kDVelocity, Constants.kTimeoutMs);
		rightMotor1.config_kF(Constants.kPIDLoopIdx, Constants.kFVelocity, Constants.kTimeoutMs);
		rightMotor1.config_IntegralZone(Constants.kPIDLoopIdx, Constants.kVelocityIZone, Constants.kTimeoutMs);
		
		rightMotor1.enableVoltageCompensation(true);
		rightMotor1.configVoltageCompSaturation(12.0, Constants.kTimeoutMs);
		rightMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kTimeoutMs);
		rightMotor1.configVelocityMeasurementWindow(1, Constants.kTimeoutMs);
		rightMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 100);
		rightMotor1.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kTimeoutMs);
		
		leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		leftMotor1.setSensorPhase(false); // keep encoder and motor in phase
		
		// set the peak, nominal outputs
		leftMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
		leftMotor1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		leftMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		leftMotor1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		// set closed loop gains in slot0 
		leftMotor1.config_kP(Constants.kPIDLoopIdx, Constants.kPVelocity, Constants.kTimeoutMs);
		leftMotor1.config_kI(Constants.kPIDLoopIdx, Constants.kIVelocity, Constants.kTimeoutMs);
		leftMotor1.config_kD(Constants.kPIDLoopIdx, Constants.kDVelocity, Constants.kTimeoutMs);
		leftMotor1.config_kF(Constants.kPIDLoopIdx, Constants.kFVelocity, Constants.kTimeoutMs);
		leftMotor1.config_IntegralZone(Constants.kPIDLoopIdx, Constants.kVelocityIZone, Constants.kTimeoutMs);
		
		leftMotor1.enableVoltageCompensation(true);
		leftMotor1.configVoltageCompSaturation(12.0, Constants.kTimeoutMs);
		leftMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kTimeoutMs);
		leftMotor1.configVelocityMeasurementWindow(1, Constants.kTimeoutMs);
		leftMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 100);
		leftMotor1.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kTimeoutMs);
	}
	
	/**
	 * Takes a velocity in inches per second and makes it the setpoint of the left motor
	 * 
	 * @param vel: the new velocity setpoint (in/sec)
	 */
	public void setLeftVel(double vel) {
		// Add acceleration term
		double accel = (vel - this.getLeftVel()) / Constants.kDefaultDt;
		vel += accel * Constants.kAVelocity;
				
		// convert velocity from inches/sec to pulses/100ms
		double targetVel = vel * Constants.kSensorUnitsPerInch / 10;
		
		// set the current setpoint of the talon's PIDF Controller to the desired velocity
		leftMotor1.set(ControlMode.Velocity, targetVel);
	}
	
	/**
	 * Takes a velocity in inches per second and makes it the setpoint of the right motor
	 * 
	 * @param vel: the new velocity setpoint (in/sec)
	 */
	public void setRightVel(double vel) {
		// Add acceleration term
		double accel = (vel - this.getRightVel()) / Constants.kDefaultDt;
		vel += accel * Constants.kAVelocity;
		
		// convert velocity from inches/sec to pulses/100ms
		double targetVel = vel * Constants.kSensorUnitsPerInch / 10;
		
		// set the current setpoint of the talon's PIDF Controller to the desired velocity
		rightMotor1.set(ControlMode.Velocity, targetVel);
	}
	

	/**
	 * Set the current position to zero on the wheel encoders
	 */
	public void zeroEncoders() {
		this.leftMotor1.setSelectedSensorPosition(0, 0, 0);
		this.rightMotor1.setSelectedSensorPosition(0, 0, 0);
	}
	
	/**
	 * Get the position of the left motors
	 * 
	 * @return: position in inches
	 */
	public double getLeftPos() {
		return this.leftMotor1.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerInch;
	}
	
	/**
	 * Get the velocity of the left motors
	 * 
	 * @return velocity in inches/sec
	 */
	public double getLeftVel() {
		return this.leftMotor1.getSelectedSensorVelocity(0) / Constants.kSensorUnitsPerInch * 10;
	}
	
	/**
	 * Get the position of the right motors
	 * 
	 * @return: position in inches
	 */
	public double getRightPos() {
		return this.rightMotor1.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerInch;
	}
	
	/**
	 * Get the velocity of the right motors
	 * 
	 * @return velocity in inches/sec
	 */
	public double getRightVel() {
		return this.rightMotor1.getSelectedSensorVelocity(0) / Constants.kSensorUnitsPerInch * 10;
	}
	
	/**
	 * Get the velocity of the robot as a whole
	 * 
	 * @return velocity in inches/sec
	 */
	public double getVel() {
		return this.getRightVel() + this.getLeftVel() / 2;
	}
	
	public void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            rightMotor1.setNeutralMode(mode);
            rightMotor2.setNeutralMode(mode);

            leftMotor1.setNeutralMode(mode);
            leftMotor2.setNeutralMode(mode);
        }
    }
}