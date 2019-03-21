package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
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
    }

    @Override
    protected void execute() {
        if (!Robot.elevator.canMoveGrabber()) {
            System.out.println("cannot move grabber");
            return;
        }
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
        if (Robot.elevator.canMoveGrabber()) {
            System.out.println("CANNOT MOVE GRABBER");
        }
        return !Robot.elevator.canMoveGrabber() || Robot.grabber.getState() == desiredPos;
    }

}