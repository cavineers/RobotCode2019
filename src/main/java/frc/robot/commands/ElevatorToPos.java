package org.usfirst.frc.team4541.robot.commands;

import frc.team4541.robot.OI.TRIG_MODE;
import frc.team4541.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
/**
 *
 */
public class ElevatorToPos extends Command {
	double targetHeight = 0;

	public ElevatorToPos(double theight) {
		requires(Robot.elevator);
		this.targetHeight = theight;
	}

	protected void initialize() {
		Robot.elevator.setVel(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double error = this.setpoint - Robot.elevator.getElevatorPos();

		
	}

	protected void interrupted() {
		end();
	}

	protected boolean isFinished() {
		double error = this.setpoint - Robot.elevator.getElevatorPos();
    	return error <= Constants.kElevatorPosTolerance; //TODO: tune
	}

	protected void end() {
		Robot.elevator.setVel(0);
	}

}
