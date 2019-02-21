package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import com.revrobotics.ControlType;

public class RawSetGrabberPosition extends Command {

    double pos;
	public RawSetGrabberPosition(double pos) {
        this.pos = pos;
		requires(Robot.grabber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.grabber.getArmMotor().getPIDController().setReference(pos, ControlType.kPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

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
	}
}
