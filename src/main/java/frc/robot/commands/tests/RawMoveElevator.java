package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import com.revrobotics.ControlType;
import frc.robot.OI.BUTTON_MODE;

public class RawMoveElevator extends Command {

    double vel;
	public RawMoveElevator(double vel) {
        this.vel = vel;
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.elevator.getElevatorMotor().set(vel);
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
        Robot.elevator.getElevatorMotor().set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.elevator.getElevatorMotor().set(0);
	}
}
