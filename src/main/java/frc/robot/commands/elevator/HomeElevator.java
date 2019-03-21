package frc.robot.commands.elevator;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class HomeElevator extends Command {

    boolean startWithLimitSwitch = false;
    boolean didFinish = false;

    public HomeElevator() {
        requires(Robot.elevator);
    }

    @Override
    protected void initialize() {
        this.setTimeout(4);
        //Disable Cascading PID Control while homing
        Robot.elevator.getPIDPos().disable();

        startWithLimitSwitch = Robot.elevator.getLimitSwitch();

        if (!startWithLimitSwitch) { //if the limit switch isn't pressed, try to move the elevator to ground
            System.out.println("Moving elevator to ground level...");
            Robot.elevator.moveElevator(ElevatorLevel.GROUND);
            Robot.elevator.getPIDPos().enable();
        }
    }

    @Override
    protected void execute() {
        //if the limit switch was pressed when the command started, move up until it is no longer pressed 
        //(gets rid of slack in the elevator line)
        if (startWithLimitSwitch) {
            System.out.println("Started with triggered limit switch; moving up...");
            Robot.elevator.getElevatorMotor().set(0.2);
            startWithLimitSwitch = Robot.elevator.getLimitSwitch();

            if (!startWithLimitSwitch) {
                System.out.println("Limit switch no longer triggered; moving down...");
                // the elevator has just lifted off of the limit switch, start going downward
                Robot.elevator.getElevatorMotor().set(-0.1);
            }

            return;
        }

        //if the grabber gets to ground level and the limit switch isn't pressed, go down until the it is pressed
        if (Robot.elevator.getLevel() == ElevatorLevel.GROUND && !Robot.elevator.getLimitSwitch()) {
            System.out.println("Elevator reached ground w/o triggering limit switch; moving down more...");
            Robot.elevator.getPIDPos().disable();
            Robot.elevator.getElevatorMotor().set(-0.1);
        }

        //the limit switch is pressed, and it was not before
        if (Robot.elevator.getLimitSwitch()) {
            System.out.println("Limit switch was pressed - grabber is homed");
            Robot.elevator.getPIDPos().disable();
            Robot.elevator.getElevatorMotor().set(0);
            Robot.elevator.setEncoderPosition(Constants.kElevatorHomeHeightRotations);
            Robot.elevator.moveElevator(ElevatorLevel.GROUND);
            didFinish = true;
        }
        
    }

    @Override
    protected void end() {
        System.out.println("Home Finished");
        //Enable Cascading PID Control when homing finishes
        Robot.elevator.getPIDPos().enable();
    }

    @Override
    protected boolean isFinished() {
        // end the command if it times out or the elevator is homed
        return isTimedOut() || didFinish;
    }

    protected void interrupted() {
        //Enable Cascading PID Control when homing is interrupted
        Robot.elevator.getPIDPos().enable();
    }
    
    
}