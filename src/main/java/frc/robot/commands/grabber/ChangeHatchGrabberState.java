package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.HatchGrabberState;

public class ChangeHatchGrabberState extends Command {
    HatchGrabberState desiredState;

    public ChangeHatchGrabberState(HatchGrabberState state) {
        desiredState = state;
    }

    @Override
    protected void initialize() {
        Robot.grabber.setHatchGrabberState(desiredState);
    }

    @Override
    protected boolean isFinished() {
        return Robot.grabber.getHatchGrabberState() == desiredState;
    }

}