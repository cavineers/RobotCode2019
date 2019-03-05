package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.MotorState;

public class ChangeGrabberState extends Command {
    GrabberPosition desiredPos;
    MotorState desiredMotorState;
    
    public ChangeGrabberState(GrabberPosition desiredPos, MotorState desiredMotorState) {
        requires(Robot.elevator);
        requires(Robot.grabber);
        this.desiredPos = desiredPos;
        this.desiredMotorState = desiredMotorState;
    }

    public ChangeGrabberState(GrabberPosition desiredPos) {
        this(desiredPos, null);
    }

    public ChangeGrabberState(MotorState desiredMotorState) {
        this(null, desiredMotorState);
    }

    @Override
    protected void initialize() {
        if(!Robot.grabber.getBallVelPID().isEnabled()){
            Robot.grabber.getBallVelPID().enable();
        }
        if (!Robot.elevator.canMoveGrabber()) {
            return;
        }
        if (this.desiredPos != null) {
            Robot.grabber.setState(desiredPos);
        }
        
        if (this.desiredMotorState != null) {
            Robot.grabber.setBallMotorState(desiredMotorState);
        }
    }

    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
        Robot.grabber.getBallVelPID().disable();
    }

    @Override
    protected boolean isFinished() {
        return Robot.grabber.getState() == desiredPos || !Robot.elevator.canMoveGrabber();
    }

}