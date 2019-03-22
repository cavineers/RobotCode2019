package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorLevel;

public class HomeElevator extends Command {

    //boolean startWithLimitSwitch = false;
    boolean isHomed = false;
    int step;

    public HomeElevator() {
        requires(Robot.elevator);
    }

    @Override
    protected void initialize() {
        this.setTimeout(10);
        //Disable Cascading PID Control while homing
        Robot.elevator.getPIDPos().disable();
        Robot.elevator.setHomed(false);
        
        if (Robot.elevator.getLimitSwitch()) { //if the limit switch is pressed, step 1 move off switch
        	Robot.elevator.getElevatorMotor().set(0.2); //move slow up
        	System.out.println("Started with triggered limit switch; moving up...");
        	step =1;  // moving up off of limit switch if started on limit switch
        } else {  //if limit switch not pressed start moving down
        	//System.out.println("Moving elevator to ground level...");
            //Robot.elevator.moveElevator(ElevatorLevel.GROUND);
            //Robot.elevator.getPIDPos().enable();
        	Robot.elevator.getElevatorMotor().set(-0.1);  //move slow down
        	step = 2;
        }
    }

    @Override
    protected void execute() {
        //if the limit switch was pressed when the command started, move up until it is no longer pressed 
        //(gets rid of slack in the elevator line)
        if (step ==1 && !Robot.elevator.getLimitSwitch()) {
        	System.out.println("Limit switch no longer triggered; moving down...");
            // the elevator has just lifted off of the limit switch, start going downward
            Robot.elevator.getElevatorMotor().set(-0.1);
            step = 2;
        }
        
        // if I am moving down and the limit switch is pressed, then I am homed 
        if (step ==2 && Robot.elevator.getLimitSwitch()){
        	System.out.println("Limit switch was pressed - Elevator is homed");
            Robot.elevator.getElevatorMotor().stopMotor();
            Robot.elevator.setEncoderPosition(Constants.kElevatorHomeHeightRotations);       
            Robot.elevator.setHomed(true);
            isHomed = true;
        }
    }

    @Override
    protected void end() {
        if (isHomed){
        	System.out.println("Home Finished");
        	//  Set setpoint & Enable Cascading PID Control
        	Robot.elevator.moveElevator(ElevatorLevel.GROUND);
        	Robot.elevator.getPIDPos().enable();
        }
    }

    @Override
    protected void interrupted() {
        Robot.elevator.getElevatorMotor().stopMotor();
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || isHomed;
    }
}