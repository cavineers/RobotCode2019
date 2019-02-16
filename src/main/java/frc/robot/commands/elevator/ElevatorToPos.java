package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI.BUTTON_MODE;
/**
 *
 */
public class ElevatorToPos extends Command {
	double targetHeight;

	public ElevatorToPos(double theight) {
		requires(Robot.elevator);
		this.targetHeight = theight;
	}

	protected void initialize() {
		Robot.elevator.getPIDPos().setSetpoint(targetHeight);
		if (!Robot.elevator.getPIDPos().isEnabled()) {
			Robot.elevator.getPIDPos().enable();
		}

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
		
	}

	protected void interrupted() {
		end();
	}

	protected boolean isFinished() {
		double upTrig = Robot.oi.getJoystick().getRawAxis(3);
		double downTrig = Robot.oi.getJoystick().getRawAxis(2);
		return Robot.elevator.getPIDPos().onTarget() || ((upTrig > 0.05 || downTrig > 0.05) && Robot.oi.currentTriggerSetting == BUTTON_MODE.ELEVATOR);
	}

	protected void end() {
	}

}
