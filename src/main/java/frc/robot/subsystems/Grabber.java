package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Constants;

public class Grabber extends Subsystem {
    public enum GrabberState {
        EXTENDED,
        RETRACTED,
        START_POS,
        HOMED,
        UNKNOWN
    }
    public GrabberState grabberState;

    CANSparkMax armMotor; // motor responcible moving the arm forward and backward

    DoubleSolenoid grabberSol; // double solenoid for control of both the 
                               // ball's endstop and the hatch grabber

    double positionOffset;

    public Grabber() {
        grabberState = GrabberState.UNKNOWN;
        armMotor = new CANSparkMax(RobotMap.armMotor, MotorType.kBrushless);
        grabberSol = new DoubleSolenoid(RobotMap.grabber1, RobotMap.grabber2);
        positionOffset = 0;
    }

    public void beginHoming() {
        armMotor.set(Constants.kGrabberHomingSpeed);
    }

    public void setPosition(GrabberState state) {
        if (state == GrabberState.EXTENDED) {
            armMotor.getPIDController().setReference(positionOffset + Constants.kGrabberExtendedPos, ControlType.kPosition);
        } else if (state == GrabberState.RETRACTED) {
            armMotor.getPIDController().setReference(positionOffset + Constants.kGrabberRetractedPos, ControlType.kPosition);
        } else if (state == GrabberState.START_POS) {
            armMotor.getPIDController().setReference(positionOffset + Constants.kGrabberStartPos, ControlType.kPosition);
        } else {
            System.out.println("Attepted to set the grabber to an ineligible position");
        }
    }

    public boolean exceedsCurrentLimit() {
        return armMotor.getOutputCurrent() > Constants.kGrabberMaxHomingCurrent; 
    }

    public void setEncoderPosition(double newPos) {
        positionOffset = armMotor.getEncoder().getPosition() - newPos;
    }

    public double getPosition() {
        return armMotor.getEncoder().getPosition() - positionOffset;
    }

    public GrabberState getState() {
        if (Math.abs(this.getPosition() - Constants.kGrabberExtendedPos) < Constants.kGrabberTolerance) {
            return GrabberState.EXTENDED;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberRetractedPos) < Constants.kGrabberTolerance) {
            return GrabberState.RETRACTED;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberStartPos) < Constants.kGrabberTolerance) {
            return GrabberState.START_POS;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberHomePos) < Constants.kGrabberTolerance) {
            return GrabberState.HOMED;
        } else {
            return GrabberState.UNKNOWN;
        }
    }

    @Override
    protected void initDefaultCommand() {
    }

}
