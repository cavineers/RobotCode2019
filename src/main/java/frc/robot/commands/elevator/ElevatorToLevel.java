package frc.robot.commands.elevator;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class ElevatorToLevel extends Command {

    public ElevatorLevel desiredLevel;
    
    public ElevatorToLevel(ElevatorLevel desiredLevel) {
        requires(Robot.elevator);
        this.desiredLevel = desiredLevel;
    }

    @Override
    protected void initialize() {
        if(!Robot.elevator.getPIDPos().isEnabled()){
            Robot.elevator.getPIDPos().enable();
        }
    }

    @Override
    protected void execute() {
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
        return Robot.elevator.getLevel() == this.desiredLevel;
    }

}