package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Write;
import java.io.PrintWriter;
import java.io.FileNotFoundException;
/**
 *
 */
public class ElevatorToPos extends Command {
    double targetHeight = 0;
    PrintWriter file = null;

	public ElevatorToPos(double targetHeight) {
		requires(Robot.elevator);
		this.targetHeight = targetHeight;
	}

	protected void initialize() {
        Robot.elevator.setVel(0);
        Robot.elevator.getElevatorTalon().setSelectedSensorPosition(0);
        file = Write.createCsvFile("C://Users//ljgre//OneDrive//Desktop//test.csv"); 
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute(){
		double error = this.targetHeight - Robot.elevator.getElevatorPos();
		double motorSpeed = Robot.elevator.getVelTrapezoid().update(Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0), error);	
        double motorSpeedInSec = (motorSpeed/Constants.pulsesPerInch)*1000;
        
        Robot.elevator.getElevPID().setSetpoint(motorSpeedInSec);
        Robot.elevator.getElevPID().enable();
        
        Write.writeCsvFile(file, ((int)motorSpeedInSec), ((int)Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0));    
        SmartDashboard.putNumber("Encoder Value", Robot.elevator.getElevatorPos());
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("Speed from VelTrap", motorSpeedInSec);
        SmartDashboard.putNumber("Actual Speed", Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0));
    }

	protected void interrupted() {
        Robot.elevator.setVel(0);
		end();
	}

	protected boolean isFinished() {
		double error = this.targetHeight - Robot.elevator.getElevatorPos();
        return error <= Constants.kElevatorPosTolerance; //TODO: tune
	}

	protected void end() {
        //TODO: fix so elevator doesn't drift down
        //Robot.elevator.setVel(0);
        file.close();
	}

}
