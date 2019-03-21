package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.MotorState;

public class ChangeGrabberState extends Command {
    GrabberPosition desiredPos;
    MotorState desiredMotorState;
    
    public ChangeGrabberState(GrabberPosition desiredPos, MotorState desiredMotorState) {
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
        if (this.desiredPos == GrabberPosition.RETRACTED && Robot.grabber.hasHatch()) {
            this.desiredPos = GrabberPosition.EXTENDED;
            this.desiredMotorState = MotorState.OFF;
        }
    }

    @Override
    protected void execute() {
        if (this.desiredPos != null && Robot.elevator.canMoveGrabber() && !Robot.grabber.hasHatch()) {
            Robot.grabber.setState(desiredPos);
        } else {  //did not reach
            System.out.println("desired: "+ this.desiredPos + "can move: " + Robot.elevator.canMoveGrabber() + "has hatch: " + Robot.grabber.hasHatch());
        }
        
        if (this.desiredMotorState != null) {
            Robot.grabber.setBallMotorState(desiredMotorState);
        }
    }

    @Override
    protected void end() {
    }

    @Override
    protected boolean isFinished() {
        return Robot.grabber.getState() == desiredPos && Robot.grabber.getBallMotorState() == desiredMotorState;
    }

}