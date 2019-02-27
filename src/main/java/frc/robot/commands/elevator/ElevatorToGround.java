package frc.robot.commands.elevator;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class ElevatorToGround extends Command {

    int step = 0;

    public ElevatorToGround() {
        requires(Robot.elevator);
        requires(Robot.grabber);
    }

    @Override
    protected void initialize() {
        if(!Robot.grabber.getState().equals(GrabberPosition.EXTENDED)){
            Robot.grabber.setState(GrabberPosition.EXTENDED);
        }
    }

    @Override
    protected void execute() {
        switch (step) {
            case 0:
                Robot.elevator.moveElevator(Constants.kElevatorGroundCheck);
                step = 1;
                break;
            case 1:
                if (Robot.elevator.getLimitSwitch()) {
                    Robot.elevator.setEncoderPosition(Constants.kElevatorHomeHeight);
                    Robot.elevator.moveElevator(Constants.kElevatorGroundLvl);
                    step = 2;
                }
                break;
        }
    }

    @Override
    protected void end() {
        Robot.elevator.setHomed(true);
    }

    @Override
    protected boolean isFinished() {
        return ((Robot.elevator.getLevel() == ElevatorLevel.GROUND || step == 2 || isTimedOut()) && (!Robot.elevator.getLimitSwitch()));
    }

    protected void interrupted() {
		Robot.elevator.setHomed(false);
    }
    
    
}