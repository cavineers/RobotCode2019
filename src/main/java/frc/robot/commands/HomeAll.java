package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorToGround;
import frc.robot.commands.grabber.HomeGrabber;

public class HomeAll extends CommandGroup {
    public HomeAll() {
        addSequential(new ElevatorToGround());
        addSequential(new HomeGrabber());
    }
}