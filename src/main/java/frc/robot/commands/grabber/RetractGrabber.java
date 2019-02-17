package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberState;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;

public class RetractGrabber extends CommandGroup {

    public RetractGrabber() {
        requires(Robot.grabber);
        addSequential(new ChangeGrabberState(GrabberState.RETRACTED));
    }
    
    @Override
    protected void initialize() {
        if (isFinished()) {
            new Rumble(0.25, ControllerSide.BOTH).start();
        }
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