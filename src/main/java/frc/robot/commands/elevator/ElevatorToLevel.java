package frc.robot.commands.elevator;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class ElevatorToLevel extends Command {

    public ElevatorLevel desiredLevel;

    Command elevToGroundCmd = null;

    public ElevatorToLevel(ElevatorLevel desiredLevel) {
        this.desiredLevel = desiredLevel;
        requires(Robot.elevator);
        requires(Robot.grabber);
    }

    @Override
    protected void initialize() {
        if (this.desiredLevel == ElevatorLevel.GROUND) {
            //if the elevator isn't already homed, or it's in the wrong spot, go to the ground position
            if (!Robot.elevator.getHomed() || Robot.elevator.getLevel() != ElevatorLevel.GROUND) {
                Robot.elevator.setHomed(false);
                elevToGroundCmd = new ElevatorToGround();
                elevToGroundCmd.start();
                return;
            }

            //the elevator is already in the desired state, finish the command
            this.cancel();
            return;
        }

        //the elevator is going to a non-ground position; move the grabber outwards, then move the elevator
        if(!Robot.grabber.getState().equals(GrabberPosition.EXTENDED)){
            Robot.grabber.setState(GrabberPosition.EXTENDED);
        }

        if(!Robot.elevator.getPIDPos().isEnabled()){
            Robot.elevator.getPIDPos().enable();
        }
    }

    @Override
    protected void execute() {
        //if the elevator is trying to go down, don't do anything
        if (elevToGroundCmd != null) {
            return;
        }
        
        //set the elevator to the desired state if the grabber is extended
        if (Robot.grabber.getState() == Grabber.GrabberPosition.EXTENDED) {
            Robot.elevator.moveElevator(this.desiredLevel);
        }
    }

    @Override
    protected void end() {
    }

    @Override
    protected boolean isFinished() {
        if (elevToGroundCmd == null) {
            return Robot.elevator.getLevel() == desiredLevel;
        } else {
            return Robot.elevator.getHomed();
        }
        
    }

    protected void interrupted() {
		end();
    }

    
    
}