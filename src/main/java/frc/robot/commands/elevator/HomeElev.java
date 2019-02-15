package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import frc.robot.Constants;
import frc.robot.Robot;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 *
 */
public class HomeElev extends Command {
    LinearDigitalFilter filter;
    int step = 0;
    boolean homed = false;
    int count = 0;

    PIDSource currentSource = new PIDSource() {
        PIDSourceType vel_sourceType = PIDSourceType.kDisplacement;

        @Override
        public double pidGet() {
            return Robot.elevator.getElevatorMotor().getOutputCurrent();
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            vel_sourceType = pidSource;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return vel_sourceType;
        }

    };
    
    LinearDigitalFilter averageCurrent = filter.movingAverage(currentSource, Constants.kHomeEncoderCurrentCycle);


	public HomeElev() {
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.getPIDPos().reset();
        Robot.elevator.getElevatorMotor().setIdleMode(IdleMode.kBrake);
        Robot.elevator.getElevatorMotor().set(Constants.kHomeMotorSpeed);
		setTimeout(Constants.kHomeTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        
		if ((Robot.elevator.getElevatorMotor().getEncoder().getVelocity() <= Constants.kHomeEncoderVelTolerance) && (Robot.elevator.getElevatorMotor().getOutputCurrent() >= (averageCurrent.get() + Constants.kHomeCurrentThreshold))) {
            Robot.elevator.setEncoderPosition(Constants.kElevatorHomeHeight);
            Robot.elevator.getPIDPos().setSetpoint(0);
            Robot.elevator.getElevatorMotor().stopMotor();
            homed = true;

		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return homed || isTimedOut();

	}

	// Called once after isFinished returns true
	protected void end() {
		homed = false;
		Robot.elevator.elevatorMotor.stopMotor();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
    }

}
