package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class ElevatorToLevel extends Command {

    public ElevatorLevel desiredLevel;

    public ElevatorToLevel(ElevatorLevel desiredLevel) {
        this.desiredLevel = desiredLevel;
        requires(Robot.elevator);
        requires(Robot.grabber);
    }

    @Override
    protected void initialize() {
        if(!Robot.grabber.getState().equals(GrabberPosition.EXTENDED)){
            Robot.grabber.setState(GrabberPosition.EXTENDED);
        }
    }

    @Override
    protected void execute() {
        if (Robot.grabber.getState() == Grabber.GrabberPosition.EXTENDED) {
            Robot.elevator.moveElevator(this.desiredLevel);
        }
    }

    @Override
    protected void end() {
    }

    @Override
    protected boolean isFinished() {
        return Robot.elevator.getLevel() == desiredLevel;
        //TODO: add interruptible if driver tries to rehome
    }

    protected void interrupted() {
		end();
	}
}