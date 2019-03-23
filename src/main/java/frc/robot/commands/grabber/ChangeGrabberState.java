package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.MotorState;

public class ChangeGrabberState extends Command {
    GrabberPosition desiredPosRaw;
    MotorState desiredMotorStateRaw;

    GrabberPosition desiredPos;
    MotorState desiredMotorState;

    
    public ChangeGrabberState(GrabberPosition desiredPos, MotorState desiredMotorState) {
        requires(Robot.grabber);
        this.desiredPosRaw = desiredPos;
        this.desiredMotorStateRaw = desiredMotorState;
        System.out.println("constructor desired: "+ desiredPos + "can move: " + Robot.elevator.canMoveGrabber() + "has hatch: " + Robot.grabber.hasHatch());

    }

    public ChangeGrabberState(GrabberPosition desiredPos) {
        this(desiredPos, null);
    }

    public ChangeGrabberState(MotorState desiredMotorState) {
        this(null, desiredMotorState);
    }

    @Override
    protected void initialize() {
        System.out.println("initial desired: "+ this.desiredPos + "can move: " + Robot.elevator.canMoveGrabber() + "has hatch: " + Robot.grabber.hasHatch());

        this.desiredPos = this.desiredPosRaw;
        this.desiredMotorState = this.desiredMotorStateRaw;

        if (this.desiredPosRaw == GrabberPosition.RETRACTED && Robot.grabber.hasHatch()) {
            System.out.println("has hatch!! canceled!!");
            this.desiredPos = GrabberPosition.EXTENDED;
            this.desiredMotorState = MotorState.OFF;
        }
        if (this.desiredMotorState != null) {
            Robot.grabber.setBallMotorState(desiredMotorState);
        }
        if(!Robot.grabber.pidPos.isEnabled()){
            Robot.grabber.pidPos.enable();
        }
    }

    @Override
    protected void execute() {
        System.out.println("running...");
        if (this.desiredPos != null && Robot.elevator.canMoveGrabber()) {
            System.out.println("desired pos " + desiredPos);
            Robot.grabber.setState(desiredPos);
        } else {  //did not reach
            System.out.println("could not move desired: "+ this.desiredPos + "can move: " + Robot.elevator.canMoveGrabber() + "has hatch: " + Robot.grabber.hasHatch());
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
        return (Robot.grabber.getState() == desiredPos || desiredPos == null) && (Robot.grabber.getBallMotorState() == desiredMotorState || desiredMotorState == null);
    }

}