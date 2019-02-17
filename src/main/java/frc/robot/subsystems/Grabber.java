package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Constants;

public class Grabber extends Subsystem {
    public enum GrabberPosition {
        EXTENDED,
        RETRACTED,
        START_POS,
        HOMED,
        UNKNOWN
    }

    public enum MotorState {
        OFF,
        INTAKE_BALL,
        EJECT_BALL
    }

    public enum HatchGrabberState {
        OPEN, //can hold hatches, but cannot receive them
        CLOSED //cannot hold hatches, but can receive them
    }

    public GrabberPosition grabberState;

    CANSparkMax armMotor; // motor responcible moving the arm forward and backward
    WPI_TalonSRX ballMotor; // motor responcible for minipulating the ball in the grabber

    DoubleSolenoid grabberSol; // double solenoid for control of both the 
                               // ball's endstop and the hatch grabber

    double positionOffset; // a correction value used to offset position from homing

    DigitalInput cargoLimitSwitch = new DigitalInput(Constants.kGrabberCargoLimitSwitch);
    DigitalInput hatchLimitSwitch = new DigitalInput(Constants.kGrabberHatchLimitSwitch);

    public Grabber() {
        grabberState = GrabberPosition.UNKNOWN;
        armMotor = new CANSparkMax(RobotMap.armMotor, MotorType.kBrushless);
        grabberSol = new DoubleSolenoid(RobotMap.grabber1, RobotMap.grabber2);
        positionOffset = 0;
    }

    /**
     * Starts moving the arm motor forwards at a given percent output for homing
     */
    public void beginHoming() {
        armMotor.set(Constants.kGrabberHomingSpeed);
    }

    /**
     * returns true if the grabber exceeds the current limit from constants
     */
    public boolean exceedsCurrentLimit() {
        return armMotor.getOutputCurrent() > Constants.kGrabberMaxHomingCurrent; 
    }

    /**
     * Sets the position offset of the grabber such that newPos is the current position
     */
    public void setEncoderPosition(double newPos) {
        positionOffset = armMotor.getEncoder().getPosition() - newPos;
    }

    /**
     * Gets the current position of the grabber, with corrections from homing (in rotations)
     */
    public double getPosition() {
        return armMotor.getEncoder().getPosition() - positionOffset;
    }

    /**
     * Sets the current state of the grabber
     * @param state the desired state of the grabber
     */
    public void setState(GrabberPosition state) {
        if (state == GrabberPosition.EXTENDED) {
            armMotor.getPIDController().setReference(positionOffset + Constants.kGrabberExtendedPos, ControlType.kPosition);
        } else if (state == GrabberPosition.RETRACTED) {
            armMotor.getPIDController().setReference(positionOffset + Constants.kGrabberRetractedPos, ControlType.kPosition);
        } else if (state == GrabberPosition.START_POS) {
            armMotor.getPIDController().setReference(positionOffset + Constants.kGrabberStartPos, ControlType.kPosition);
        } else {
            System.out.println("Attepted to set the grabber to an ineligible position");
        }
    }

    /**
     * Get the current state of the grabber
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
     * @param state the desired state of the grabber
     */
    public void setMotorState(MotorState state) {
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
        return this.cargoLimitSwitch.get();
    }

    /**
     * Returns if the limit switch for hatches on the grabber is currently pressed 
     */
    public boolean hasHatch() {
        return this.hatchLimitSwitch.get();
    }

    @Override
    protected void initDefaultCommand() {
    }

    public CANSparkMax getGrabberMotor(){
        return armMotor;
    }


}
