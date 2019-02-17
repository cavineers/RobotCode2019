package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class HomeGrabber extends Command {
    boolean isHomed;
    public HomeGrabber() {
        requires(Robot.grabber);
        requires(Robot.elevator);
        isHomed = false;
    }

    @Override
    public void initialize() {
        if (Robot.elevator.canMoveGrabber()) {
            Robot.grabber.beginHoming();
        } else {
            new Rumble(0.25, ControllerSide.BOTH).start();
            this.cancel();
        }
        isHomed = false;
    }

    @Override
    public void execute() {
        if (Robot.grabber.exceedsCurrentLimit() && !isHomed && Robot.elevator.canMoveGrabber()) {
            Robot.grabber.setEncoderPosition(Constants.kGrabberHomePos);
            isHomed = true;
        }
    }

    @Override
    public void end() {
        Robot.grabber.setState(GrabberPosition.EXTENDED);
    }

    public boolean isFinished() {
        return isHomed || !Robot.elevator.canMoveGrabber();
    }
}