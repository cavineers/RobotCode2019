package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.VelocityTrapezoid;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorToPos;

/**
 * The Elevator subsystem 
 */
public class Elevator extends Subsystem {
    //TODO: add two sparkmax
	public WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(RobotMap.elevatorMotor1);
    
    VelocityTrapezoid velTrapezoid = new VelocityTrapezoid(Constants.kMaxAccelSpeedUp, Constants.kMaxTurnToAngleSpeed, Constants.kDefaultDt);
    DigitalInput limitSwitch;

    public Elevator() {
        limitSwitch = new DigitalInput(0); // change input later
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevatorMotor.setSensorPhase(true); /* keep sensor and motor in phase */
    }

    /**
     * Initialize normal driving as a default command
     */
    @Override
    public void initDefaultCommand() {
		setDefaultCommand(new ElevatorToPos(10));
    }

    /**
     * get the talon of elevator motor
     * @return talon of elevator motor
     */
    public WPI_TalonSRX getElevatorTalon() {
        return this.elevatorMotor;
    }
    
    public double getElevatorPos() {
		return getElevatorTalon().getSelectedSensorPosition(0);
	}

    public DigitalInput getLimitSwitch() {
		return limitSwitch;
	}

	// public void setManualVelocity(double trigger) {
	// 	manualVelocity = trigger;
	// }
	
	public double getElevatorVel() {
		return elevatorMotor.getSelectedSensorVelocity(0);
	}

    public void setVel(double vel) {
        // set the current setpoint of the talon's PIDF Controller to the desired velocity
        // elevatorMotor.set(ControlMode.Velocity, vel);
    }

    public VelocityTrapezoid getVelTrapezoid(){
        return this.velTrapezoid;
    }
}