package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
/**
 *
 */
public class ElevatorToPos extends Command {
    double targetHeight = 0;
    //PrintWriter file = null;
    int slot = 0;

	public ElevatorToPos(double targetHeight) {
		requires(Robot.elevator);
		this.targetHeight = targetHeight;
	}

	protected void initialize() {
        Robot.elevator.setVel(0);
        Robot.elevator.getElevatorTalon().setSelectedSensorPosition(0);
        //file = Write.createCsvFile("C://Users//ljgre//OneDrive//Desktop//LOOK.csv"); 
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute(){
        double error = this.targetHeight - Robot.elevator.getCurrentHeight();
		double motorSpeedInSec = Robot.elevator.getVelTrapezoid().update(Robot.elevator.getVelInchesPerSecond(), error);	
        double motorSpeedPulses = Elevator.convertToPulsesPer100Ms(motorSpeedInSec);
        
        System.out.println(error + "," + motorSpeedInSec + "," + Robot.elevator.getVelInchesPerSecond());

        /*Robot.elevator.getElevPID().setSetpoint(motorSpeedInSec);
        Robot.elevator.getElevPID().enable();*/

        Robot.elevator.getElevatorTalon().set(ControlMode.Velocity, motorSpeedPulses);
        // System.out.println((Integer.toString((int)motorSpeedPulses)) + "," + (Integer.toString(Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0))));

        //Write.writeCsvFile(file, ((int)motorSpeedInSec), ((int)Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0)));    
        SmartDashboard.putNumber("Talon Error", Robot.elevator.getElevatorTalon().getClosedLoopError(0));
        SmartDashboard.putNumber("Encoder Value", Robot.elevator.getElevatorTalon().getSelectedSensorPosition());
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("Speed from VelTrap", motorSpeedPulses);
        SmartDashboard.putNumber("Actual Speed", Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0));
    }

	protected void interrupted() {
        Robot.elevator.setVel(0);
		end();
	}

	protected boolean isFinished() {
		/*double error = this.targetHeight - ((Robot.elevator.getElevatorPos()/Constants.pulsesPerInch)*1000);
        System.out.println(error);
        return error <= Constants.kElevatorPosTolerance; //TODO: tune*/
        return false;
	}

	protected void end() {
        //TODO: fix so elevator doesn't drift down
        //Robot.elevator.setVel(0);
        //file.close();
	}

}
