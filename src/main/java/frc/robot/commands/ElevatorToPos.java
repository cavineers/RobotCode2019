package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import com.revrobotics.ControlType;
import frc.robot.OI.TRIG_MODE;

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
		Robot.elevator.getElevatorMotor().getPIDController().setReference(targetHeight, ControlType.kVelocity);
		if (!Robot.elevator.getPIDAccel().isEnabled()) {
			Robot.elevator.getPIDAccel().enable();
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
		return Robot.elevator.getPIDAccel().onTarget() || ((upTrig > 0.05 || downTrig > 0.05) && Robot.oi.currentTriggerSetting == TRIG_MODE.ELEVATOR);
//		return true;
	}

	protected void end() {
	}

}
