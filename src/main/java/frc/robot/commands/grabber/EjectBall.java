package frc.robot.commands.grabber;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Grabber.MotorState;

public class EjectBall extends Command{
    private boolean finished;

    public EjectBall() {
        
    }

    @Override
    protected void initialize() {
        requires(Robot.grabber);
        Robot.grabber.setMotorState(MotorState.EJECT_BALL);
        this.setTimeout(Constants.kGrabberEjectionTime);
    }

    @Override
    protected void end() {
        Robot.grabber.setMotorState(MotorState.OFF);
    }

    @Override
    protected boolean isFinished() {
        return this.finished;
    }
}