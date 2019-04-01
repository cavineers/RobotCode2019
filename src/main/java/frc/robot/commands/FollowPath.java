package frc.robot.commands;

import frc.lib.pathPursuit.ArcSegment;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Point;
import frc.lib.pathPursuit.RobotCmd;
import frc.lib.RobotPos;
import frc.lib.pathPursuit.Segment;
import frc.robot.AutoPathHelper;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.AutoPathHelper.PATH_TYPE;

import edu.wpi.first.wpilibj.command.Command;

public class FollowPath extends Command {
    public Path path;

    public FollowPath(PATH_TYPE pathType) {
        this.path = AutoPathHelper.getPath(pathType);
        requires(Robot.drivetrain);
    }

    public FollowPath(Path pathName) {
        this.path = pathName;
        requires(Robot.drivetrain);
    }
    
    @Override
    protected void initialize() {
        Robot.drivetrain.setLeftVel(0);
        Robot.drivetrain.setRightVel(0);
    }
    
    @Override
    public void execute() {
        RobotPos latestPos = Robot.estimator.getPos();
        latestPos.setVelocities(Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel());

        RobotCmd cmd = this.path.update(latestPos);
        // System.out.println("runnning path!");
        //debug print: lTarget, rTarget, lActualVel, rActualVel, RobotPosition
//		System.out.println(Robot.drivetrain.leftMotor1.getClosedLoopTarget(0) +  "," + Robot.drivetrain.rightMotor1.getClosedLoopTarget(0) + "," + Robot.drivetrain.leftMotor1.getSelectedSensorVelocity(0) + "," + Robot.drivetrain.rightMotor1.getSelectedSensorVelocity(0) + "," + Robot.state.getPosition());
        // System.out.println(latestPos.getX() + ", " + latestPos.getY());
        Robot.drivetrain.setLeftVel(cmd.getLeftVel());
        Robot.drivetrain.setRightVel(cmd.getRightVel());
    }
    
    @Override
    protected boolean isFinished() {
        return this.path.isFinished();
    }
    
    @Override
    protected void end() {
        Robot.drivetrain.setLeftVel(0);
        Robot.drivetrain.setRightVel(0);
        if (this.isFinished()) {
            System.out.println("FINISHED PATH");
            System.out.println(Robot.estimator.getPos());
        }
    }

}
