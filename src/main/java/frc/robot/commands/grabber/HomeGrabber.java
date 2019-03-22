package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class HomeGrabber extends Command {
    boolean startedAtHome = false;
    boolean isHomed = false;
    
    public HomeGrabber() {
        requires(Robot.grabber);
        this.setTimeout(10);
    }

    @Override
    public void initialize() {
        isHomed = false;
        Robot.grabber.setHomed(false);
        startedAtHome = Robot.grabber.isAtHome();
        Robot.grabber.pidPos.disable();
    }

    @Override
    public void execute() {
        if (startedAtHome && Robot.grabber.isAtHome()) {
            //move the grabber forward enough that the limit switch is not pressed
            Robot.grabber.getArmMotor().set(-0.1);
            return;
        }

        if (startedAtHome && !Robot.grabber.isAtHome()) {
            startedAtHome = false;
        }
        Robot.grabber.getArmMotor().set(0.1);

        if (Robot.grabber.isAtHome()) {
            System.out.println("homed!");
            Robot.grabber.setEncoderPosition(Constants.kGrabberHomePos);
            Robot.grabber.setState(GrabberPosition.EXTENDED);
            isHomed = true;
            Robot.grabber.setHomed(true);
        }
    }

    @Override
    public void end() {
        if (this.isHomed) {
            Robot.grabber.setEncoderPosition(Constants.kGrabberHomePos);
            Robot.grabber.setState(GrabberPosition.EXTENDED);
            Robot.grabber.pidPos.enable();
        } else {
            Robot.grabber.getArmMotor().set(0);;
        }
    }

    @Override
    public void interrupted() {
        Robot.grabber.getArmMotor().disable();
        Robot.grabber.getArmMotor().set(0);;
    }

    public boolean isFinished() {
        return isHomed || !Robot.elevator.canMoveGrabber() || this.isTimedOut();
    }
}