package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberState;

public class ExtendGrabber extends CommandGroup {

    public ExtendGrabber() {
        requires(Robot.grabber);
        addSequential(new ChangeGrabberState(GrabberState.EXTENDED));
    }

    @Override
    public void end() {

    }

    @Override
    protected void interrupted() {
        this.end(); //make sure that the end method is called even if the command is interrupted / canceled 
    }
}