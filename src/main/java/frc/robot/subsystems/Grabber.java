package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.lib.MathHelper;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PIDOutput;
import com.revrobotics.CANSparkMax.IdleMode;

public class Grabber extends Subsystem {
    public enum GrabberPosition {
        EXTENDED, RETRACTED, START_POS, HOMED, UNKNOWN
    }

    public enum MotorState {
        OFF, INTAKE_BALL, EJECT_BALL
    }

    public enum HatchGrabberState {
        HOLDING, // can hold hatches, but cannot receive them
        INTAKING // cannot hold hatches, but can receive them
    }

    public GrabberPosition grabberState;

    CANSparkMax armMotor; // motor responcible moving the arm forward and backward
    public WPI_TalonSRX ballMotor; // motor responcible for manipulating the ball in the grabber

    public DoubleSolenoid grabberSol; // double solenoid for control of the hatch grabber

    //limit switches for checking what game piece is in the grabber
    DigitalInput cargoLimitSwitch = new DigitalInput(RobotMap.kGrabberCargoLimitSwitch);
    DigitalInput hatchLimitSwitch = new DigitalInput(RobotMap.kGrabberHatchLimitSwitch);

    //limit switch for homing the grabber
    DigitalInput homingLimitSwitch = new DigitalInput(RobotMap.kGrabberHomeingLimitSwitch);

    //pid controller for intaking / ejecting the ball
    PIDController ballVelPID;

    //pid controller for controling the entire grabber's position
    public PIDController pidPos;

    double lastToggleTime = 0;

    boolean isHomed = false;

    public Grabber() {
        grabberState = GrabberPosition.UNKNOWN;
        armMotor = new CANSparkMax(RobotMap.armMotor, MotorType.kBrushless);
        armMotor.setInverted(true);
        grabberSol = new DoubleSolenoid(RobotMap.PCM1, RobotMap.grabber1, RobotMap.grabber2);
        ballMotor = new WPI_TalonSRX(RobotMap.grabberIntake);

        this.getArmMotor().getPIDController().setP(Constants.kGrabberVelP);
        this.getArmMotor().getPIDController().setI(Constants.kGrabberVelI);
        this.getArmMotor().getPIDController().setD(Constants.kGrabberVelD);
        this.getArmMotor().getPIDController().setFF(Constants.kGrabberVelF);
        this.getArmMotor().getPIDController().setOutputRange(-1, 1);
        this.getArmMotor().setIdleMode(IdleMode.kBrake);

        pidPos = new PIDController(Constants.kGrabberPosP, Constants.kGrabberPosI, Constants.kGrabberPosD, Constants.kGrabberPosF, new PIDSource() {
            PIDSourceType vel_sourceType = PIDSourceType.kDisplacement;

            @Override
            public double pidGet() {
                return getArmMotor().getEncoder().getPosition();
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
            public void pidWrite(double vel) {
                getArmMotor().getPIDController().setReference(vel, ControlType.kVelocity);
            }

        }, Constants.kGrabberPIDPosPeriod);

        pidPos.enable();

        pidPos.setInputRange(Constants.kGrabberRetractedPos, 0);
        pidPos.setOutputRange(-Constants.kElevatorMaxSpeed, Constants.kElevatorMaxSpeed);
        pidPos.setContinuous(false);
        pidPos.setPercentTolerance(Constants.kElevPercentTolerance);

        ballVelPID = new PIDController(Constants.kGrabberBallVelP, Constants.kGrabberBallVelI, Constants.kGrabberBallVelD, Constants.kGrabberBallVelF, new PIDSource() {
			PIDSourceType out_sourceType = PIDSourceType.kRate;

			@Override
			public double pidGet() {
				return ballMotor.getSelectedSensorVelocity(0);
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
                ballMotor.set(ControlMode.PercentOutput, d);
            }
        }, Constants.kGrabberBallPeriod);

        this.setHatchGrabberState(HatchGrabberState.HOLDING);

    }

    /**
     * Sets the position offset of the grabber such that newPos is the current
     * position
     */
    public void setEncoderPosition(double newPos) {
        armMotor.getEncoder().setPosition(newPos);
        
    }

    /**
     * Gets the current position of the grabber, with corrections from homing (in
     * rotations)
     */
    public double getPosition() {
        return armMotor.getEncoder().getPosition();
    }

    /**
     * Sets the current state of the grabber
     * 
     * @param state the desired state of the grabber
     */
    public void setState(GrabberPosition state) {
        if (state == GrabberPosition.EXTENDED) {
            this.pidPos.setSetpoint(Constants.kGrabberExtendedPos);
        } else if (state == GrabberPosition.RETRACTED) {
            this.pidPos.setSetpoint(Constants.kGrabberRetractedPos);
        } else if (state == GrabberPosition.START_POS) {
            this.pidPos.setSetpoint(Constants.kGrabberStartPos);
        } else {
            System.out.println("Attepted to set the grabber to an ineligible position");
        }
    }

    /**
     * Get the current state of the grabber
     * 
     * @return the current state of the grabber
     */
    public GrabberPosition getState() {
        if (Math.abs(this.getPosition() - Constants.kGrabberExtendedPos) < Constants.kGrabberTolerance) {
            return GrabberPosition.EXTENDED;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberRetractedPos) < Constants.kGrabberTolerance) {
            return GrabberPosition.RETRACTED;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberStartPos) < Constants.kGrabberTolerance) {
            return GrabberPosition.START_POS;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberHomePos) < Constants.kGrabberTolerance) {
            return GrabberPosition.HOMED;
        } else {
            return GrabberPosition.UNKNOWN;
        }
    }
    
    public PIDController getBallVelPID(){
        return ballVelPID;
    }

    /**
     * Sets the current state of the grabber
     * 
     * @param state the desired state of the grabber
     */
    public void setBallMotorState(MotorState state) {
        if (state == MotorState.OFF) {
            this.getBallVelPID().reset();
            this.ballMotor.set(0);
        } else if (state == MotorState.INTAKE_BALL) {
            if (!this.getBallVelPID().isEnabled()) {
                this.getBallVelPID().enable();
            }
            this.getBallVelPID().setSetpoint(Constants.kGrabberIntakeSpeed);
        } else if (state == MotorState.EJECT_BALL) {
            if (!this.getBallVelPID().isEnabled()) {
                this.getBallVelPID().enable();
            }
            this.getBallVelPID().setSetpoint(Constants.kGrabberEjectionSpeed);
        }
    }

    public MotorState getBallMotorState() {
        if (this.ballMotor.get() == 0) {
            return MotorState.OFF;
        } else if (MathHelper.areApproxEqual(this.getBallVelPID().getSetpoint(), Constants.kGrabberIntakeSpeed)) {
            return MotorState.INTAKE_BALL;
        } else if (MathHelper.areApproxEqual(this.getBallVelPID().getSetpoint(), Constants.kGrabberEjectionSpeed)) {
            return MotorState.EJECT_BALL;
        }

        return MotorState.OFF;
    }

    /**
     * Sets the current state of the hatch grabber
     */
    public void setHatchGrabberState(HatchGrabberState state) {
        if (state == HatchGrabberState.HOLDING) {
            this.grabberSol.set(Value.kForward);
        } else {
            this.grabberSol.set(Value.kReverse);
        }
    }

    /**
     * Gets the current state of the hatch grabber
     */
    public HatchGrabberState getHatchGrabberState() {
        if (this.grabberSol.get() == Value.kReverse) {
            return HatchGrabberState.INTAKING;
        } else {
            return HatchGrabberState.HOLDING;
        }
    }

    /**
     * Returns true if the limit switch for cargo on the grabber is currently pressed
     */
    public boolean hasCargo() {
        return !this.cargoLimitSwitch.get();
    }

    /**
     * Returns true if the limit switch for hatches on the grabber is currently pressed
     */
    public boolean hasHatch() {
        return !this.hatchLimitSwitch.get();
    }

    /**
     * Returns true if the limit switch meant for homing the grabber is currently pressed
     */
    public boolean isAtHome() {
        return !this.homingLimitSwitch.get();
    }

    @Override
    protected void initDefaultCommand() {
    }

    public CANSparkMax getArmMotor() {
        return armMotor;
    }

    public TalonSRX getBallMotor(){
        return ballMotor;
    }

    public double getLastToggleTime(){
        return this.lastToggleTime;
    }

    public void setLastToggleTime(double newToggleTime){
        this.lastToggleTime = newToggleTime;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setHomed(boolean homed) {
        this.isHomed = homed;
    }

    

}
