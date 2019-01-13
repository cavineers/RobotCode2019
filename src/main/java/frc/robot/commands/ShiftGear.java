package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.DriveGear;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShiftGear extends Command {
    DriveGear gear;
    public ShiftGear(DriveGear gear) {
        this.gear = gear;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.drivetrain.setDriveGear(gear);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}