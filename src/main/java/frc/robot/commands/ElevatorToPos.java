package frc.robot.commands;

import frc.robot.OI.TRIG_MODE;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
/**
 *
 */
public class ElevatorToPos extends Command {
	double targetHeight = 0;

	public ElevatorToPos(double targetHeight) {
		requires(Robot.elevator);
		this.targetHeight = targetHeight;
	}

	protected void initialize() {
		Robot.elevator.setVel(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double error = this.targetHeight - Robot.elevator.getElevatorPos();

		
	}

	protected void interrupted() {
		end();
	}

	protected boolean isFinished() {
		double error = this.targetHeight - Robot.elevator.getElevatorPos();
        return error <= Constants.kElevatorPosTolerance; //TODO: tune
	}

	protected void end() {
		Robot.elevator.setVel(0);
	}

}
