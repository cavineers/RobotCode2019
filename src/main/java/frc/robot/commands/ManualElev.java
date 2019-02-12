package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import com.revrobotics.ControlType;
import frc.robot.OI.TRIG_MODE;



/**
 *
 */
public class ManualElev extends Command {
	double elevPos;
	double upTrig;
	double downTrig;
	boolean changePos = false;

	public ManualElev() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (!Robot.elevator.getPIDAccel().isEnabled()) {
			elevPos = Robot.elevator.getCurrentHeight();
			Robot.elevator.getPIDAccel().setSetpoint(elevPos);
            Robot.elevator.getElevatorMotor().getPIDController().setReference(0, ControlType.kPosition);
			Robot.elevator.getPIDAccel().enable();

		}

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		upTrig = Robot.oi.getJoystick().getRawAxis(3);
		downTrig = Robot.oi.getJoystick().getRawAxis(2);
		if (upTrig > 0.05 && downTrig < 0.05 && Robot.oi.currentTriggerSetting == TRIG_MODE.ELEVATOR) {
			Robot.elevator.setManualVelocity(Constants.kElevatorMaxSpeed * Math.pow(upTrig, 2));
			Robot.elevator.getPIDAccel().setSetpoint(Constants.kElevatorMaxHeight);
			// Robot.elevator.updatePIDVals();
		}

		else if (downTrig > 0.05 && upTrig < 0.05 && Robot.oi.currentTriggerSetting == TRIG_MODE.ELEVATOR) {
			Robot.elevator.setManualVelocity(-1 * (Constants.kElevatorMaxSpeed * Math.pow(downTrig, 2)));
			Robot.elevator.getPIDAccel().setSetpoint(Constants.kElevatorMinHeight);

		}

		else {
			Robot.elevator.setManualVelocity(0);

			if (!Robot.elevator.getPIDAccel().onTarget()) {
				Robot.elevator.getPIDAccel().setSetpoint(Robot.elevator.getElevatorMotor().getEncoder().getPosition());
				System.out.println("not on target");
			}
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.elevator.setManualVelocity(9999);
	}
}
