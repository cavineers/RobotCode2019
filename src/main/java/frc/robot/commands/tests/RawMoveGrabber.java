package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import com.revrobotics.ControlType;
import frc.robot.OI.BUTTON_MODE;

public class RawMoveGrabber extends Command {

    double vel;
	public RawMoveGrabber(double vel) {
        this.vel = vel;
		requires(Robot.grabber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.grabber.getArmMotor().set(vel);
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
        Robot.grabber.getArmMotor().set(0);
	}
}
