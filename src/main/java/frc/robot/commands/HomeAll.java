package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.elevator.ElevatorToGround;
import frc.robot.commands.grabber.HomeGrabber;

public class HomeAll extends CommandGroup {
    public HomeAll() {
        addSequential(new Command() {
            @Override
            protected void initialize() {
                Robot.grabber.pidPos.disable();
                Robot.grabber.getArmMotor().getPIDController().setReference(0, ControlType.kVelocity);        
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
        });
        addSequential(new ElevatorToGround());
        addSequential(new HomeGrabber());
    }
}