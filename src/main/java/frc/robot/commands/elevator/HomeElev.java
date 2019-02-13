package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 *
 */
public class HomeElev extends Command {

	int step = 0;

	public HomeElev() {
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.getPIDPos().reset();
		Robot.elevator.getElevatorMotor().setIdleMode(IdleMode.kBrake);
		Robot.elevator.getElevatorMotor().set(.7);
		step = 1;
		setTimeout(20);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		switch (step) {
		case 1:
			if (Robot.elevator.getElevatorMotor().getEncoder().getPosition() >= (Constants.kElevatorPulsesPerInch*2)) {
				Robot.elevator.getElevatorMotor().set(-.1);
				step = 2;
			}
			break;

		case 2:
			if (!Robot.elevator.getLimitSwitch().get()) {
				Robot.elevator.getElevatorMotor().set(0);
				Robot.elevator.getPIDPos().setSetpoint(0);
				Robot.elevator.getElevatorMotor().stopMotor();
				step = 3;
			}
			break;

		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return step == 3 || isTimedOut();

	}

	// Called once after isFinished returns true
	protected void end() {
		
		Robot.elevator.elevatorMotor.stopMotor();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

}
