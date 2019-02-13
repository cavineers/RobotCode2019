package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberState;

public class HomeGrabber extends Command {
    boolean isHomed;
    public HomeGrabber() {
        requires(Robot.grabber);
        requires(Robot.elevator);
        isHomed = false;
    }

    @Override
    public void initialize() {
        Robot.grabber.beginHoming();
        isHomed = false;
    }

    @Override
    public void execute() {
        if (Robot.grabber.exceedsCurrentLimit() && !isHomed) {
            Robot.grabber.setEncoderPosition(Constants.kGrabberHomePos);
            isHomed = true;
        }
    }

    @Override
    public void end() {
        Robot.grabber.setState(GrabberState.EXTENDED);
    }

    public boolean isFinished() {
        return isHomed;
    }
}