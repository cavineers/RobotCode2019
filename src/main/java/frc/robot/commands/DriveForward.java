package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class DriveForward extends TimedCommand {
    double velocity;

    public DriveForward(double velocity, double timeout) {
        super(timeout);
        requires(Robot.drivetrain);
        this.velocity = velocity;
    }

    protected void initialize() {
        Robot.drivetrain.drive(velocity, 0);
    }

    protected void end() {
        Robot.drivetrain.drive(0, 0);
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

}