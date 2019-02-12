package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class GrabberFindHome extends Command {

    public GrabberFindHome() {
        requires(Robot.grabber);
    }

    @Override
    public void initialize() {
        Robot.grabber.findEnd();
    }

    @Override
    public void execute() {
        if (Robot.grabber.currentLimit()) {
            Robot.grabber.zero();
        }
    }

    public boolean isFinished() {
        return !Robot.grabber.getHoming();
    }
}