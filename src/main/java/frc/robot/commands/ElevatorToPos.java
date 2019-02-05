package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;
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
        Robot.elevator.getElevatorTalon().configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        Robot.elevator.getElevatorTalon().config_kP(Constants.kPIDLoopIdx, Constants.kPVelocityElev);
        Robot.elevator.getElevatorTalon().config_kI(Constants.kPIDLoopIdx, Constants.kIVelocityElev);
        Robot.elevator.getElevatorTalon().config_kD(Constants.kPIDLoopIdx, Constants.kDVelocityElev);
        Robot.elevator.getElevatorTalon().config_kF(Constants.kPIDLoopIdx, Constants.kFVelocityElev);
        Robot.elevator.getElevatorTalon().enableVoltageCompensation(true);
        Robot.elevator.getElevatorTalon().configVoltageCompSaturation(12.0, Constants.kTimeoutMs);
        Robot.elevator.getElevatorTalon().configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kTimeoutMs);
        Robot.elevator.getElevatorTalon().configVelocityMeasurementWindow(1, Constants.kTimeoutMs);
        Robot.elevator.getElevatorTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 100);
        //file = Write.createCsvFile("C://Users//ljgre//OneDrive//Desktop//LOOK.csv"); 
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute(){
		double error = this.targetHeight - (Robot.elevator.getElevatorPos()/Constants.pulsesPerInch);
		double motorSpeedInSec = Robot.elevator.getVelTrapezoid().update((Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0)*Constants.pulsesPerInch)/10, error);	
        double motorSpeed = (motorSpeedInSec*Constants.pulsesPerInch)/10;
        
        /*Robot.elevator.getElevPID().setSetpoint(motorSpeedInSec);
        Robot.elevator.getElevPID().enable();*/

        Robot.elevator.getElevatorTalon().set(ControlMode.Velocity, motorSpeed);
        System.out.println((Integer.toString((int)motorSpeed)) + "," + (Integer.toString(Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0))));

        //Write.writeCsvFile(file, ((int)motorSpeedInSec), ((int)Robot.elevator.getElevatorTalon().getSelectedSensorVelocity(0)));    
        SmartDashboard.putNumber("Talon Error", Robot.elevator.getElevatorTalon().getClosedLoopError(0));
        SmartDashboard.putNumber("Encoder Value", Robot.elevator.getElevatorPos());
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("Speed from VelTrap", motorSpeed);
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
