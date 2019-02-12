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
        HALFWAY,
        HOMING,
        UNKOWN
    }
    public double backPosition;
    public GrabberState grabberState;
    public boolean finishedHoming;

    CANSparkMax armMotor; // motor responcible moving the arm forward and backward

    DoubleSolenoid grabberSol; // double solenoid for control of both the 
                               // ball's endstop and the hatch grabber

    public Grabber() {
        grabberState = GrabberState.UNKOWN;
        armMotor = new CANSparkMax(RobotMap.armMotor, MotorType.kBrushless);
        grabberSol = new DoubleSolenoid(RobotMap.grabber1, RobotMap.grabber2);
        finishedHoming = false;
    }

    public void findEnd() {
        grabberState = GrabberState.HOMING;
        finishedHoming = false;
        armMotor.set(Constants.kGrabberHomingSpeed);
    }

    public void center() {
        grabberState = GrabberState.HALFWAY;
        armMotor.getPIDController().setReference(backPosition + 0, ControlType.kPosition);
    }

    public void extend() {
        grabberState = GrabberState.EXTENDED;
        armMotor.getPIDController().setReference(backPosition + 0, ControlType.kPosition);
    }

    public void retract() {
        grabberState = GrabberState.RETRACTED;
        armMotor.getPIDController().setReference(backPosition + 0, ControlType.kPosition);
    }

    public boolean currentLimit() {
        if (armMotor.getOutputCurrent() > 1.0) { // If amps is creater than 10
            armMotor.getPIDController().setReference(0, ControlType.kVoltage);
            grabberState = GrabberState.RETRACTED; // Set the state to off
            finishedHoming = true;
            return true;
        } else {
            return false;
        }
    }

    public void zero() {
        backPosition = armMotor.getEncoder().getPosition();
    }

    public boolean getHoming() {
        return !finishedHoming;
    }

    @Override
    protected void initDefaultCommand() {
    }

}
