package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.HatchGrabberState;
import frc.robot.subsystems.Grabber.MotorState;

public class ToggleHatchGrabber extends Command {

    public ToggleHatchGrabber() {
        requires(Robot.grabber);
    }

    @Override
    protected void initialize() {
        if (Robot.grabber.getState() != GrabberPosition.EXTENDED) {
            this.cancel();
        } else {
            Robot.grabber.setLastToggleTime(Robot.getCurrentTime());
            if (Robot.grabber.getHatchGrabberState() == HatchGrabberState.OPEN) {
                Robot.grabber.setHatchGrabberState(HatchGrabberState.CLOSED);
            } else {
                Robot.grabber.setHatchGrabberState(HatchGrabberState.OPEN);
            }
        }
    }

    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }


}