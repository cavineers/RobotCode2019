package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;

public class Grabber extends Subsystem {
    public enum GrabberPosition {
        EXTENDED, RETRACTED, START_POS, HOMED, UNKNOWN
    }

    public enum MotorState {
        OFF, INTAKE_BALL, EJECT_BALL
    }

    public enum HatchGrabberState {
        OPEN, // can hold hatches, but cannot receive them
        CLOSED // cannot hold hatches, but can receive them
    }

    public GrabberPosition grabberState;

    CANSparkMax armMotor; // motor responcible moving the arm forward and backward
    public WPI_TalonSRX ballMotor; // motor responcible for minipulating the ball in the grabber

    public DoubleSolenoid grabberSol; // double solenoid for control of the hatch grabber

    DigitalInput cargoLimitSwitch = new DigitalInput(Constants.kGrabberCargoLimitSwitch);
    DigitalInput hatchLimitSwitch = new DigitalInput(Constants.kGrabberHatchLimitSwitch);

    LinearDigitalFilter averageCurrent;

    public Grabber() {
        grabberState = GrabberPosition.UNKNOWN;
        armMotor = new CANSparkMax(RobotMap.armMotor, MotorType.kBrushless);
        grabberSol = new DoubleSolenoid(RobotMap.PCM1, RobotMap.grabber1, RobotMap.grabber2);
        ballMotor = new WPI_TalonSRX(RobotMap.grabberIntake);

        this.getArmMotor().getPIDController().setP(Constants.kGrabberPosP);
        this.getArmMotor().getPIDController().setI(Constants.kGrabberPosI);
        this.getArmMotor().getPIDController().setD(Constants.kGrabberPosD);
        this.getArmMotor().getPIDController().setFF(Constants.kGrabberPosF);
        this.getArmMotor().getPIDController().setOutputRange(-1, 1);
        this.getArmMotor().setIdleMode(IdleMode.kBrake);

    }

    /**
     * Starts moving the arm motor forwards at a given percent output for homing
     */
    public void beginHoming() {
        armMotor.set(Constants.kGrabberHomingSpeed);
        averageCurrent = LinearDigitalFilter.movingAverage(Robot.grabber.getGrabberCurrent(),
                Constants.kHomeEncoderCurrentCycle);
    }

    /**
     * returns true if the grabber exceeds the current limit from constants
     */
    public boolean exceedsCurrentLimit() {
        return Robot.grabber.getArmMotor().getOutputCurrent() >= Constants.kGrabberMaxHomingCurrent;
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
            this.getArmMotor().getPIDController().setReference(Constants.kGrabberExtendedPos,
                    ControlType.kPosition);
        } else if (state == GrabberPosition.RETRACTED) {
            this.getArmMotor().getPIDController().setReference(Constants.kGrabberRetractedPos,
                    ControlType.kPosition);
        } else if (state == GrabberPosition.START_POS) {
            this.getArmMotor().getPIDController().setReference(Constants.kGrabberRetractedPos,
                    ControlType.kPosition);
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

    /**
     * Sets the current state of the grabber
     * 
     * @param state the desired state of the grabber
     */
    public void setBallMotorState(MotorState state) {
        if (state == MotorState.OFF) {
            this.ballMotor.set(0);
        } else if (state == MotorState.INTAKE_BALL) {
            this.ballMotor.set(Constants.kGrabberIntakeSpeed);
        } else if (state == MotorState.EJECT_BALL) {
            this.ballMotor.set(Constants.kGrabberEjectionSpeed);
        }
    }

    /**
     * Sets the current state of the hatch grabber
     */
    public void setHatchGrabberState(HatchGrabberState state) {
        if (state == HatchGrabberState.CLOSED) {
            this.grabberSol.set(Value.kForward);
        } else {
            this.grabberSol.set(Value.kReverse);
        }
    }

    /**
     * Gets the current state of the hatch grabber
     */
    public HatchGrabberState getHatchGrabberState() {
        if (this.grabberSol.get() == Value.kForward) {
            return HatchGrabberState.CLOSED;
        } else {
            return HatchGrabberState.OPEN;
        }
    }

    /**
     * Returns if the limit switch for cargo on the grabber is currently pressed
     */
    public boolean hasCargo() {
        return !this.cargoLimitSwitch.get();
    }

    /**
     * Returns if the limit switch for hatches on the grabber is currently pressed
     */
    public boolean hasHatch() {
        return !this.hatchLimitSwitch.get();
    }

    @Override
    protected void initDefaultCommand() {
    }

    public CANSparkMax getArmMotor() {
        return armMotor;
    }

    public PIDSource getGrabberCurrent() {
        return new PIDSource() {

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {

            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return getArmMotor().getOutputCurrent();
            }

        };
    }


}
