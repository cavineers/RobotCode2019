package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import com.revrobotics.ControlType;

public class RawSetGrabberVelocity extends Command {

    double vel;
	public RawSetGrabberVelocity(double vel) {
        this.vel = vel;
		requires(Robot.grabber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        Robot.grabber.getArmMotor().getPIDController().setReference(this.vel, ControlType.kVelocity);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
        Robot.grabber.getArmMotor().set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.grabber.getArmMotor().set(0);
	}
}
