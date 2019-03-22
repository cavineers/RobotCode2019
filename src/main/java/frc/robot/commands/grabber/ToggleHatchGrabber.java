package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.HatchGrabberState;
import frc.robot.subsystems.Grabber.MotorState;

public class ToggleHatchGrabber extends Command {
    public ToggleHatchGrabber() {
        // requires(Robot.grabber);
    }

    @Override
    protected void initialize() {
        // if (Robot.grabber.getState() == GrabberPosition.EXTENDED) {
        Robot.grabber.setLastToggleTime(Robot.getCurrentTime());
        if (Robot.grabber.getHatchGrabberState() == HatchGrabberState.HOLDING) {
            Robot.grabber.setHatchGrabberState(HatchGrabberState.INTAKING);
        } else {
            Robot.grabber.setHatchGrabberState(HatchGrabberState.HOLDING);
        }
        // }
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