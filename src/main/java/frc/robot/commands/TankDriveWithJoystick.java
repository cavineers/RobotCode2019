package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Have the robot drive tank style using the XBox Joystick until interrupted.
 */
public class TankDriveWithJoystick extends Command {

    public TankDriveWithJoystick() {
        requires(Robot.drivetrain);
    }
    @Override
    protected void initialize() {
        Robot.drivetrain.getRightTalon().set(ControlMode.Velocity, 0);
        Robot.drivetrain.getLeftTalon().set(ControlMode.Velocity, 0);
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.drivetrain.drive(Robot.oi.getJoystick());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drivetrain.drive(0, 0);
    }
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
