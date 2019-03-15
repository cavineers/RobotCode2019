package frc.robot.commands.elevator;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class ElevatorToGround extends Command {

    int step = 0;
    boolean startWithLimitSwitch = false;

    public ElevatorToGround() {
        requires(Robot.elevator);
    }

    @Override
    protected void initialize() {
        startWithLimitSwitch = Robot.elevator.getLimitSwitch();
        Robot.elevator.setHomed(false);

        if (!startWithLimitSwitch) { //if the limit switch isn't pressed, try to move the elevator to ground
            Robot.elevator.moveElevator(ElevatorLevel.GROUND);
        }
    }

    @Override
    protected void execute() {
        //if the limit switch was pressed when the command started, move up until it is no longer pressed 
        //(gets rid of slack in the elevator line)
        if (startWithLimitSwitch) {
            Robot.elevator.getElevatorMotor().set(0.2);
            startWithLimitSwitch = Robot.elevator.getLimitSwitch();

            if (!startWithLimitSwitch) {
                // the elevator has just lifted off of the limit switch, start going downward
                Robot.elevator.getElevatorMotor().set(-0.1);
            }

            return;
        }

        //if the grabber gets to ground level and the limit switch isn't pressed, go down until the it is pressed
        if (Robot.elevator.getLevel() == ElevatorLevel.GROUND && !Robot.elevator.getLimitSwitch()) {
            Robot.elevator.getElevatorMotor().set(-0.1);
        }

        //the limit switch is pressed, and it was not before
        if (Robot.elevator.getLimitSwitch()) {
            Robot.elevator.getElevatorMotor().set(0);
            Robot.elevator.setHomed(true);
            Robot.elevator.setEncoderPosition(Constants.kElevatorHomeHeight);
            Robot.elevator.moveElevator(ElevatorLevel.GROUND);
        }
        
    }

    @Override
    protected void end() {
        
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || Robot.elevator.getHomed();
    }

    protected void interrupted() {
		Robot.elevator.setHomed(false);
    }
    
    
}