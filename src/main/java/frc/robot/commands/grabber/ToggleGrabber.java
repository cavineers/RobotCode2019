package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class ToggleGrabber extends ConditionalCommand {

    public ToggleGrabber() {
        //first argument runs if condition() returns true, second one if it returns false
        super(new RetractGrabber(), new ExtendGrabber());
    }

    @Override
    protected void initialize() {
        super.initialize();
        System.out.println("toggling grabber...");
        if (!Robot.elevator.canMoveGrabber() || Robot.grabber.hasHatch()) {
            new Rumble(0.25, ControllerSide.BOTH).start();
        }
    }

    @Override
    protected boolean condition() {
        //true if grabber should retract, otherwise false
        //grabber is currently extended, it will not interfere with the elevator, and the grabber does not have a hatch, move back 
        return Robot.grabber.getState() == GrabberPosition.EXTENDED;
    }

}