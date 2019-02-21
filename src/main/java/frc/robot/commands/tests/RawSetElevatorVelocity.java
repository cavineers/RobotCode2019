package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import com.revrobotics.ControlType;

public class RawSetElevatorVelocity extends Command {

    double vel;
	public RawSetElevatorVelocity(double vel) {
        this.vel = vel;
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        Robot.elevator.setVelocity(this.vel);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
        Robot.elevator.getElevatorMotor().set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.elevator.getElevatorMotor().set(0);
	}
}
