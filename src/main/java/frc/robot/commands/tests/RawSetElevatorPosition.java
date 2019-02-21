package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import com.revrobotics.ControlType;

public class RawSetElevatorPosition extends Command {

    double pos;
	public RawSetElevatorPosition(double pos) {
        this.pos = pos;
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.elevator.getPIDPos().enable();
        Robot.elevator.getPIDPos().setSetpoint(this.pos);
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
        Robot.elevator.getPIDPos().disable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.elevator.getElevatorMotor().set(0);
	}
}
