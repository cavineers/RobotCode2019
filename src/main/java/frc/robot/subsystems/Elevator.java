package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
    //TODO: add two sparkmax
	public WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(RobotMap.elevatorMotor1);
    /*private PIDController elevPID;
    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double F = .0002;
    private double period = .025;*/

    VelocityTrapezoid velTrapezoid = new VelocityTrapezoid(Constants.kMaxElevAcceleration, Constants.kMaxElevSpeed, Constants.kDefaultDt);
    DigitalInput limitSwitch;

    public Elevator() {
        velTrapezoid.setDecelLock(false);
         /*elevPID = new PIDController(P, I, D, F, new PIDSource() {
			PIDSourceType out_sourceType = PIDSourceType.kDisplacement;

			@Override
			public double pidGet() {
				return elevatorMotor.getSelectedSensorVelocity(0);
			}

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				out_sourceType = pidSource;
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return out_sourceType;
			}
		},

				new PIDOutput() {
					@Override
					public void pidWrite(double d) {
                        SmartDashboard.putNumber("PID output", d);
						elevatorMotor.set(ControlMode.PercentOutput, d);
					}
                }, period);

                elevPID.setInputRange(-Constants.kMaxElevSpeed, Constants.kMaxElevSpeed);
                elevPID.setOutputRange(-1, 1);
                elevPID.setContinuous(false);
                elevPID.setPercentTolerance(1);
                elevPID.setSetpoint(0);*/
                
        limitSwitch = new DigitalInput(0); // change input later
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        elevatorMotor.setSensorPhase(true); /* keep sensor and motor in phase */
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
        elevatorMotor.set(ControlMode.Velocity, vel);
        //elevatorMotor.set(vel);
    }

    public VelocityTrapezoid getVelTrapezoid(){
        return this.velTrapezoid;
    }

   /* public PIDController getElevPID() {
		return elevPID;
	}*/
}