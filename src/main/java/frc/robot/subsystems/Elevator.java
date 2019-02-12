package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The Elevator subsystem 
 */
public class Elevator extends Subsystem {
	public CANSparkMax elevatorMotor = new CANSparkMax(RobotMap.elevatorMotor, MotorType.kBrushless);
    DigitalInput limitSwitch;
    private PIDController pidAccel;
    private double manualVelocity = 9999;
	private double prevOutput = 0;
	private int loop = 0;

    public Elevator() {
        limitSwitch = new DigitalInput(0); // change input later

        //zero the motor's encoder
        this.getElevatorMotor().set(0);

        // set PID coefficients
        this.getElevatorMotor().getPIDController().setP(Constants.kPVelocityElev);
        this.getElevatorMotor().getPIDController().setI(Constants.kIVelocityElev);
        this.getElevatorMotor().getPIDController().setD(Constants.kDVelocityElev);
        this.getElevatorMotor().getPIDController().setFF(Constants.kFVelocityElev);
        this.getElevatorMotor().getPIDController().setOutputRange(-1, 1);

        this.getElevatorMotor().set(-500);
        this.getElevatorMotor().setIdleMode(IdleMode.kBrake);
    

        

        pidAccel = new PIDController(Constants.kPAccelElev, Constants.kIAccelElev, Constants.kDAccelElev, new PIDSource() {
			PIDSourceType vel_sourceType = PIDSourceType.kDisplacement;

			@Override
			public double pidGet() {
				return elevatorMotor.getEncoder().getPosition();
			}

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				vel_sourceType = pidSource;
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return vel_sourceType;
			}

		}, new PIDOutput() {
			@Override
			public void pidWrite(double d) {
				// compare what PID says to trigger
				if(Math.abs(d-prevOutput) > Constants.kElevatorMaxAcceleration/4) {
					if(d>prevOutput) {
						d=prevOutput + Constants.kElevatorMaxAcceleration/4;
					}
					else {
						d=prevOutput - Constants.kElevatorMaxAcceleration/4;
					}
				}
				
				prevOutput = d;
     
				if (manualVelocity == 9999)
					elevatorMotor.getPIDController().setReference(d, ControlType.kVelocity);

				else if (manualVelocity > 0) {
					elevatorMotor.getPIDController().setReference(Math.min(d, manualVelocity), ControlType.kVelocity);
					loop = 0;
				}
				else if (manualVelocity < 0) {
					elevatorMotor.getPIDController().setReference(Math.max(d, manualVelocity), ControlType.kVelocity);
					loop = 0;
				}
				else if (manualVelocity == 0) {
					elevatorMotor.getPIDController().setReference(0, ControlType.kVelocity);
					loop++;
				}
				if(loop == 10) {
					manualVelocity = 9999;
					loop = 0;
				}

			}
		}, Constants.kElevPIDAccelPeriod);

		pidAccel.setInputRange(Constants.kElevatorMinHeight, Constants.kElevatorMaxHeight);
		pidAccel.setOutputRange(-Constants.kElevatorMaxSpeed, Constants.kElevatorMaxSpeed);
		pidAccel.setContinuous(false);
		pidAccel.setPercentTolerance(Constants.kElevPercentTolerance);
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
    public CANSparkMax getElevatorMotor() {
        return this.elevatorMotor;
    }
    
    public DigitalInput getLimitSwitch() {
		return limitSwitch;
	}

	public double getElevatorVel() {
		return this.getElevatorMotor().getEncoder().getVelocity();
	}

    public void setVel(double vel) {
        this.getElevatorMotor().set(vel);
    }

    /**
     * returns the current velocity of the elevator in inches per second
     */
    public double getVelInchesPerSecond() {
        return (this.getElevatorMotor().getEncoder().getVelocity()/Constants.kElevatorPulsesPerInch) * 10;
    }

    /**
     * returns the current height of the elevator in inches
     */
    public double getCurrentHeight() {
        return this.getElevatorMotor().getEncoder().getPosition()/Constants.kElevatorPulsesPerInch;
    }

    /**
     * Converts a given speed in inches/sec to pulses per 100ms
     */
    public static double convertToPulsesPer100Ms(double velInPerSec) {
        return (velInPerSec / 10) * Constants.kElevatorPulsesPerInch;
    }

    public PIDController getPIDAccel(){
        return pidAccel;
    }

    public void setManualVelocity(double trigger) {
		manualVelocity = trigger;
    }
    

}