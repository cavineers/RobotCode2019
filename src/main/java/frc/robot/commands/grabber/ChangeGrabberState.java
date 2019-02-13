package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberState;

public class ChangeGrabberState extends Command {
    GrabberState desiredState;
    
    public ChangeGrabberState(GrabberState desiredState) {
        requires(Robot.elevator);
        requires(Robot.grabber);
        this.desiredState = desiredState;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if (!Robot.elevator.canMoveGrabber()) {
            return;
        }
        Robot.grabber.setState(desiredState);
    }

    @Override
    protected void end() {

    }

    @Override
    protected boolean isFinished() {
        return Robot.grabber.getState() == desiredState || !Robot.elevator.canMoveGrabber();
    }

}