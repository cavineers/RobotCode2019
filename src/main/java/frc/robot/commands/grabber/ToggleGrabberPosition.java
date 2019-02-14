package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;

public class ToggleGrabberPosition extends CommandGroup {

    public ToggleGrabberPosition() {
        // addSequential(new ChangeGrabberState(desiredPos));
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || Robot.grabber.hasHatch() || !Robot.elevator.canMoveGrabber(); //TODO: finish if there is a ball already in the grabber
    }

    @Override
    public boolean isCanceled() {
        return super.isCanceled() || isFinished(); // immediately stop the command when isFinished is true
    }

    @Override
    public void end() {

    }

    @Override
    protected void interrupted() {
        this.end(); //make sure that the end method is called even if the command is interrupted / canceled 
    }
}