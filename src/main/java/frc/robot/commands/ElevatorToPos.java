package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        Robot.elevator.getElevatorTalon().setSelectedSensorPosition(0);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double error = this.targetHeight - Robot.elevator.getElevatorPos();
		double motorSpeed = Robot.elevator.getVelTrapezoid().update(Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0), error);	
        Robot.elevator.setVel(motorSpeed);
        SmartDashboard.putNumber("Encoder Value", Robot.elevator.getElevatorPos());
        SmartDashboard.putNumber("Error", error);
    }

	protected void interrupted() {
		end();
	}

	protected boolean isFinished() {
		double error = this.targetHeight - Robot.elevator.getElevatorPos();
        return error <= Constants.kElevatorPosTolerance; //TODO: tune
	}

	protected void end() {
        //TODO: fix so elevator doesn't drift down
		Robot.elevator.setVel(0);
	}

}
