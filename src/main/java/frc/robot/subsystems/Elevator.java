package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.VelocityTrapezoid;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorToPos;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * The Elevator subsystem 
 */
public class Elevator extends Subsystem {
	public WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(RobotMap.elevatorMotor);
    VelocityTrapezoid velTrapezoid = new VelocityTrapezoid(Constants.kElevatorMaxAcceleration, Constants.kElevatorMaxSpeed, Constants.kDefaultDt);
    DigitalInput limitSwitch;

    public Elevator() {
        velTrapezoid.setDecelLock(false);
        limitSwitch = new DigitalInput(0); // change input later
        elevatorMotor.setSensorPhase(true); /* keep sensor and motor in phase */

        this.getElevatorTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.getElevatorTalon().config_kP(Constants.kPIDLoopIdx, Constants.kPVelocityElev);
        this.getElevatorTalon().config_kI(Constants.kPIDLoopIdx, Constants.kIVelocityElev);
        this.getElevatorTalon().config_kD(Constants.kPIDLoopIdx, Constants.kDVelocityElev);
        this.getElevatorTalon().config_kF(Constants.kPIDLoopIdx, Constants.kFVelocityElev);
        this.getElevatorTalon().enableVoltageCompensation(true);
        this.getElevatorTalon().configVoltageCompSaturation(12.0, Constants.kTimeoutMs);
        this.getElevatorTalon().configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kTimeoutMs);
        this.getElevatorTalon().configVelocityMeasurementWindow(1, Constants.kTimeoutMs);
        this.getElevatorTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 100);

        // set the peak, nominal outputs
        this.getElevatorTalon().configNominalOutputForward(0, Constants.kTimeoutMs);
        this.getElevatorTalon().configNominalOutputReverse(0, Constants.kTimeoutMs);
        this.getElevatorTalon().configPeakOutputForward(1, Constants.kTimeoutMs);
        this.getElevatorTalon().configPeakOutputReverse(-1, Constants.kTimeoutMs);
    }

    /**
     * Initialize normal driving as a default command
     */
    @Override
    public void initDefaultCommand() {
		//setDefaultCommand(new ElevatorToPos(Constants.pulsesPerInch*10));
    }

    /**
     * get the talon of elevator motor
     * @return talon of elevator motor
     */
    public WPI_TalonSRX getElevatorTalon() {
        return this.elevatorMotor;
    }
    
    public DigitalInput getLimitSwitch() {
		return limitSwitch;
	}

	public double getElevatorVel() {
		return elevatorMotor.getSelectedSensorVelocity(0);
	}

    public void setVel(double vel) {
        elevatorMotor.set(ControlMode.Velocity, vel);
        //elevatorMotor.set(vel);
    }

    public VelocityTrapezoid getVelTrapezoid(){
        return this.velTrapezoid;
    }

    /**
     * returns the current velocity of the elevator in inches per second
     */
    public double getVelInchesPerSecond() {
        return (this.getElevatorTalon().getSelectedSensorVelocity(0)/Constants.kElevatorPulsesPerInch) * 10;
    }

    /**
     * returns the current height of the elevator in inches
     */
    public double getCurrentHeight() {
        return this.getElevatorTalon().getSelectedSensorPosition(0)/Constants.kElevatorPulsesPerInch;
    }

    /**
     * Converts a given speed in inches/sec to pulses per 100ms
     */
    public static double convertToPulsesPer100Ms(double velInPerSec) {
        return (velInPerSec / 10) * Constants.kElevatorPulsesPerInch;
    }

}